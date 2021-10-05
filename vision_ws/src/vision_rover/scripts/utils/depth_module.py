import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

    def read(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()


        color_Img = np.asanyarray(color_frame.get_data())
        align = rs.align(rs.stream.color)
        frames = align.process(frames)
        new_depth_frame = frames.get_depth_frame()

        colorizer = rs.colorizer()
        hole_filling = rs.hole_filling_filter(2)
        spatial = rs.spatial_filter()
        decimation = rs.decimation_filter()

        spatial.set_option(rs.option.filter_magnitude, 5)
        spatial.set_option(rs.option.filter_smooth_alpha, 1)
        spatial.set_option(rs.option.filter_smooth_delta, 50)
        spatial.set_option(rs.option.holes_fill, 3)

        new_depth_frame = decimation.process(new_depth_frame)
        new_depth_frame = spatial.process(new_depth_frame)
        new_depht_frame = hole_filling.process(new_depth_frame)

        colorized_depth = np.asanyarray(colorizer.colorize(new_depht_frame).get_data())

        depth_image = np.asanyarray(new_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image, colorized_depth

    def release(self):
        self.pipeline.stop()