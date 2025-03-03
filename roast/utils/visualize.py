#!/usr/bin/env python3
import logging
import threading

import gi
import open3d as o3d
from gi.repository import GLib, Gst, GstRtspServer

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")


class PointCloudVisualizer:
    def __init__(self, intrinsic_matrix, width, height):
        self.depth_map = None
        self.rgb = None
        self.pcl = None

        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width,
            height,
            intrinsic_matrix[0][0],
            intrinsic_matrix[1][1],
            intrinsic_matrix[0][2],
            intrinsic_matrix[1][2],
        )
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.isstarted = False

    def rgbd_to_projection(self, depth_map, rgb, is_rgb):
        self.depth_map = depth_map
        self.rgb = rgb
        rgb_o3d = o3d.geometry.Image(self.rgb)
        depth_o3d = o3d.geometry.Image(self.depth_map)
        # TODO: query frame shape to get this, and remove the param 'is_rgb'
        if is_rgb:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb_o3d, depth_o3d, convert_rgb_to_intensity=False
            )
        else:
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb_o3d, depth_o3d
            )
        if self.pcl is None:
            self.pcl = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, self.pinhole_camera_intrinsic
            )
        else:
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, self.pinhole_camera_intrinsic
            )
            self.pcl.points = pcd.points
            self.pcl.colors = pcd.colors
        return self.pcl

    def visualize_pcd(self):
        if not self.isstarted:
            self.vis.add_geometry(self.pcl)
            origin = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.3, origin=[0, 0, 0]
            )
            self.vis.add_geometry(origin)
            self.isstarted = True
        else:
            self.vis.update_geometry(self.pcl)
            self.vis.poll_events()
            self.vis.update_renderer()

    def close_window(self):
        self.vis.destroy_window()


class RtspSystem(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(RtspSystem, self).__init__(**properties)
        log = logging.getLogger("RTSP")
        log.info("init rtsp system")
        self.frame = None
        self.number_frames = 0
        self.fps = 15
        self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds
        self.launch_string = (
            "appsrc name=source is-live=true block=true format=GST_FORMAT_TIME "
            "caps=video/x-raw,format=BGR,width=300,height=300,framerate={}/1 "
            "! videoconvert ! video/x-raw,format=I420 "
            "! x264enc speed-preset=ultrafast tune=zerolatency "
            "! rtph264pay config-interval=1 name=pay0 pt=96".format(self.fps)
        )

    def send_frame(self, data):
        self.frame = data

    def start(self):
        t = threading.Thread(target=self._thread_rtsp)
        t.start()

    def _thread_rtsp(self):
        loop = GLib.MainLoop()
        loop.run()

    def on_need_data(self, src, length):
        # log.info("In on_need_data")
        data = self.frame.tostring()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.duration = self.duration
        timestamp = self.number_frames * self.duration
        buf.pts = buf.dts = int(timestamp)
        buf.offset = timestamp
        self.number_frames += 1
        retval = src.emit("push-buffer", buf)
        #    log.info('pushed buffer, frame {}, duration {} ns, durations {} s'.
        # format(self.number_frames,
        # self.duration,
        # self.duration / Gst.SECOND))
        if retval != Gst.FlowReturn.OK:
            print(retval)

    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name("source")
        appsrc.connect("need-data", self.on_need_data)


class RTSPServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(RTSPServer, self).__init__(**properties)
        self.rtsp = RtspSystem()
        self.rtsp.set_shared(True)
        self.get_mount_points().add_factory("/preview", self.rtsp)
        self.attach(None)
        Gst.init(None)
        self.rtsp.start()

    def send_frame(self, frame):
        self.rtsp.send_frame(frame)
