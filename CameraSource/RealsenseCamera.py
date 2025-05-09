import cv2
import pyrealsense2 as rs
import numpy as np
# Camera source


class RealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        print("Loading Intel Realsense Camera")
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame")
            return False, None, None
        
        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)
        
        # Create colormap to show the depth of the Objects
        colorizer = rs.colorizer()
        depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())

        # Convert images to numpy arrays
        # distance = depth_frame.get_distance(int(50),int(50))
        # print("distance", distance)

        # depth_convert = np.asanyarray(depth_frame.get_data())
        depth_convert = np.asanyarray(filled_depth.get_data())
        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_convert, alpha=0.5), cv2.COLORMAP_JET)
        color_image = np.asanyarray(color_frame.get_data())

        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        depth_distCoe = depth_intrinsics.coeffs
        color_distCoe = color_intrinsics.coeffs

        # cv2.imshow("Colormap", depth_colormap)
        # cv2.imshow("depth img", depth_image)

        return True, color_image, depth_image, color_intrinsics, depth_frame, color_distCoe

    def pixel_to_point(self, u, v):
        _, _, _, depth_intrinsic, depth_frame, _ = self.get_frame_stream()

        # Get distance at pixel point (u,v)
        depth_value = depth_frame.get_distance(u, v)
        # pixel = (u, v)
        # distance = depth_frame[pixel[1], pixel[0]]

        # Sử dụng API để tính toán chuyển đổi điểm ảnh (pixel) sang tọa độ 3D (point)
        X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrinsic, [u, v], (depth_value * 1000))

        return X, Y, Z, depth_value

    def release(self):
        self.pipeline.stop()
        # print(depth_image)
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.10), 2)
        # Stack both images horizontally
        # images = np.hstack((color_image, depth_colormap))
