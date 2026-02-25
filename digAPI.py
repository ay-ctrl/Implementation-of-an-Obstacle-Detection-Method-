from flask import Flask, request, jsonify
import numpy as np
import cv2
import math
import open3d as o3d
import os

app = Flask(__name__)

@app.route("/upload_stereo", methods=["POST"])
def upload_stereo():
    # --- Dosya al ---
    left_file = request.files.get("left_image")
    right_file = request.files.get("right_image")
    if not left_file or not right_file:
        return "Missing files", 400

    left_bytes = left_file.read()
    right_bytes = right_file.read()
    left_img = cv2.imdecode(np.frombuffer(left_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imdecode(np.frombuffer(right_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
    left_color = cv2.imdecode(np.frombuffer(left_bytes, np.uint8), cv2.IMREAD_COLOR)

    # --- 1) DENSE STEREO MATCHING ---
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,        # Minimum disparity value to be considered.
        numDisparities=16*3,   # Range of disparity values to search.
        blockSize=5,           # Size of the block/window used for matching.
        P1=8*3*5*5,            # Penalty on the disparity change by ±1 between neighboring pixels.
        P2=32*3*5*5,           # Penalty on the disparity change by more than 1 between neighboring pixels.
        disp12MaxDiff=1,       # Maximum allowed difference in the left-right disparity check.
        uniquenessRatio=8,      # Margin in percent by which the best match should be better than the second-best.
        speckleWindowSize=50,  # Maximum size of smooth disparity regions to consider as speckle noise.
        speckleRange=2,        # Maximum disparity variation within each connected component to detect speckles.
        preFilterCap=63,       # Truncation value for prefiltered image pixels.
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # Use 3-way semi-global matching for better accuracy.
    )
    disparity = stereo.compute(left_img, right_img).astype(np.float32)/16.0
    disparity[disparity<=0] = 0
    disparity = cv2.medianBlur(disparity,5)

    os.makedirs("debug_outputs", exist_ok=True)
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    cv2.imwrite("debug_outputs/disparity.png", disp_vis)

    # --- 2) 3D RE-PROJECTION ---
    h, w = left_img.shape
    fov_vertical_deg = 60
    aspect = w/h
    fov_rad = math.radians(fov_vertical_deg)
    fy = h/(2*math.tan(fov_rad/2))
    fx = fy*aspect
    cx, cy = w/2, h/2
    baseline = 0.06
    Q = np.float32([[1,0,0,-cx],[0,1,0,-cy],[0,0,0,fx],[0,0,1/baseline,0]])
    points_3d = cv2.reprojectImageTo3D(disparity, Q)
    Z = points_3d[:,:,2]
    mask_3d = (Z>0) & (Z<10)
    points = points_3d[mask_3d]
    colors = cv2.cvtColor(left_color, cv2.COLOR_BGR2RGB)[mask_3d]/255.0

    # Open3D point cloud kaydı
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    #o3d.visualization.draw_geometries([pcd])  

    # --- 3) MEAN-SHIFT SEGMENTATION ---
    left_bgr = cv2.cvtColor(left_img, cv2.COLOR_GRAY2BGR)
    mean_shifted = cv2.pyrMeanShiftFiltering(left_bgr, sp=15, sr=20)
    segmented_gray = cv2.cvtColor(mean_shifted, cv2.COLOR_BGR2GRAY)
    _, segmented_bin = cv2.threshold(segmented_gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    num_labels, labels = cv2.connectedComponents(segmented_bin)
    min_region_size = 50
    cleaned_labels = np.zeros_like(labels)
    current_label = 1
    for label in range(1,num_labels):
        region_mask = (labels==label)
        if np.sum(region_mask)>=min_region_size:
            cleaned_labels[region_mask] = current_label
            current_label+=1

    # Segment map kaydı
    max_label = np.max(cleaned_labels)
    label_hue = np.uint8(179*cleaned_labels/max(max_label,1))
    blank_ch = 255*np.ones_like(label_hue)
    colored_labels = cv2.merge([label_hue, blank_ch, blank_ch])
    colored_labels = cv2.cvtColor(colored_labels, cv2.COLOR_HSV2BGR)
    colored_labels[label_hue==0]=0
    cv2.imwrite("debug_outputs/segmented_regions.png", colored_labels)

    # --- 4) SEED POINTS ---
    T1 = 10
    h, w = cleaned_labels.shape
    num_regions = np.max(cleaned_labels)
    region_seed_count = np.zeros(num_regions+1, dtype=int)
    valid_mask = (Z>0) & (Z<10)
    ys, xs = np.where(valid_mask)
    for y,x in zip(ys,xs):
        label = cleaned_labels[y,x]
        if label>0:
            region_seed_count[label]+=1
    seed_region_mask = np.zeros_like(cleaned_labels, dtype=bool)
    for label in range(1,num_regions+1):
        if region_seed_count[label]>=T1:
            seed_region_mask[cleaned_labels==label]=True
    cv2.imwrite("debug_outputs/seed_regions.png", 
                (seed_region_mask.astype(np.uint8)*255))

    # --- 5) OBSTACLE DETECTION ---
    window_size = 5
    T2 = 0.0005
    T3 = 0.005
    T4 = 50
    obstacle_map = np.zeros_like(cleaned_labels, dtype=bool)
    rough_step_map = np.zeros_like(cleaned_labels, dtype=np.uint8)

    for region_label in range(1,num_regions+1):
        region_mask = (cleaned_labels==region_label)
        ys, xs = np.where(region_mask)
        if len(ys)<10: continue
        y_min, y_max = ys.min(), ys.max()
        x_min, x_max = xs.min(), xs.max()
        for y_start in range(y_min, y_max, window_size):
            for x_start in range(x_min, x_max, window_size):
                y_end = min(y_start+window_size, y_max+1)
                x_end = min(x_start+window_size, x_max+1)
                window_mask = region_mask[y_start:y_end, x_start:x_end]
                if np.sum(window_mask)<3: continue
                window_points = points_3d[y_start:y_end, x_start:x_end,:]
                window_points = window_points[window_mask]
                window_points = window_points[np.isfinite(window_points).all(axis=1)]
                if len(window_points)<3: continue
                X = window_points[:,:2]
                Z_vals = window_points[:,2].reshape(-1,1)
                A = np.hstack([X,np.ones((X.shape[0],1))])
                try:
                    coeff, _, _, _ = np.linalg.lstsq(A,Z_vals,rcond=None)
                    plane_z = A@coeff
                    roughness = np.mean(np.abs(Z_vals-plane_z))
                    step = np.max(Z_vals)-np.min(Z_vals)
                    rough_step_map[y_start:y_end, x_start:x_end] = int(min(255,(roughness+step)*100))
                    if roughness>T2 or step>T3:
                        mask_to_update = np.zeros_like(region_mask,dtype=bool)
                        mask_to_update[y_start:y_end, x_start:x_end] = True
                        obstacle_map |= (mask_to_update & region_mask)
                except np.linalg.LinAlgError:
                    continue
        if region_seed_count[region_label]<T1:
            mean_gray = np.mean(left_img[region_mask])
            if mean_gray<T4:
                obstacle_map |= region_mask

    cv2.imwrite("debug_outputs/rough_step_map.png", rough_step_map)

    # --- 6) MORPHOLOGICAL POST-PROCESSING ---
    obstacle_map_img = obstacle_map.astype(np.uint8)*255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
    obstacle_map_img = cv2.morphologyEx(obstacle_map_img, cv2.MORPH_CLOSE, kernel, iterations=2)
    obstacle_map_img = cv2.morphologyEx(obstacle_map_img, cv2.MORPH_OPEN, kernel, iterations=2)
    cv2.imwrite("debug_outputs/final_obstacle_map.png", obstacle_map_img)
    print(f"Detected obstacles in {int(np.sum(obstacle_map))} pixels.")

    return jsonify({"status":"success","num_obstacles":int(np.sum(obstacle_map))})

if __name__=="__main__":
    os.makedirs("debug_outputs", exist_ok=True)
    app.run(host="127.0.0.1", port=5000, debug=True)
