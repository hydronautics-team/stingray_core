# stingray_core_control/rqt_plugin.py
import threading
import time
from collections import deque

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QSlider, QHBoxLayout,
    QFileDialog, QCheckBox, QListWidget, QListWidgetItem, QSizePolicy
)
from python_qt_binding.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, UInt8, UInt8MultiArray
from geometry_msgs.msg import Twist

# try pyqtgraph
try:
    import pyqtgraph as pg
    HAVE_PYQTGRAPH = True
except Exception:
    HAVE_PYQTGRAPH = False

# ---- ROS client node used inside plugin ----
class RosClient(Node):
    def __init__(self,
                 topic_pressure='/sensors/pressure',
                 topic_control_data='/control/data',
                 topic_loop_flags='/control/loop_flags',
                 topic_thruster_cmd='/thruster/cmd'):
        super().__init__('rqt_stingray_client')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub_pressure = self.create_subscription(Float32, topic_pressure, self.cb_pressure, qos)
        self.sub_control_data = self.create_subscription(Twist, topic_control_data, self.cb_control_data, qos)
        self.sub_loop_flags = self.create_subscription(UInt8, topic_loop_flags, self.cb_loop_flags, qos)
        self.sub_thruster_cmd = self.create_subscription(UInt8MultiArray, topic_thruster_cmd, self.cb_thruster_cmd, qos)

        # publishers (optional) — reuse same set topics as example; you can remove if not needed
        self.pub_depth_set = self.create_publisher(Float32, '/depth/set', 10)

        self.latest_depth = None
        # impact commands from /control/data
        self.latest_impact = {
            'surge': None, 'sway': None, 'depth': None,
            'roll': None, 'pitch': None, 'yaw': None
        }
        self.latest_flags = 0
        self.latest_thruster_cmd = []

        self.get_logger().info('rqt_stingray_client started')

    def cb_pressure(self, msg: Float32):
        try:
            self.latest_depth = float(msg.data)
        except Exception as e:
            self.get_logger().warning(f"pressure parse error: {e}")

    def cb_control_data(self, msg: Twist):
        try:
            self.latest_impact['surge'] = float(msg.linear.x)
            self.latest_impact['sway'] = float(msg.linear.y)
            self.latest_impact['depth'] = float(msg.linear.z)
            self.latest_impact['roll'] = float(msg.angular.x)
            self.latest_impact['pitch'] = float(msg.angular.y)
            self.latest_impact['yaw'] = float(msg.angular.z)
        except Exception as e:
            self.get_logger().warning(f"control_data parse error: {e}")

    def cb_loop_flags(self, msg: UInt8):
        try:
            self.latest_flags = int(msg.data)
        except Exception as e:
            self.get_logger().warning(f"loop_flags parse error: {e}")

    def cb_thruster_cmd(self, msg: UInt8MultiArray):
        try:
            # store as list of ints
            self.latest_thruster_cmd = [int(x) for x in msg.data]
        except Exception as e:
            self.get_logger().warning(f"thruster_cmd parse error: {e}")

    def publish_depth_set(self, v: float):
        m = Float32(); m.data = float(v); self.pub_depth_set.publish(m)


# ---- rqt plugin ----
class StingrayEngineer(Plugin):
    def __init__(self, context):
        super(StingrayEngineer, self).__init__(context)
        self.setObjectName('StingrayEngineer')

        # init rclpy if needed
        try:
            rclpy.init()
            self._rclpy_inited_here = True
        except RuntimeError:
            self._rclpy_inited_here = False

        # create ROS client node + executor
        self.ros_node = RosClient()
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.ros_node)
        self.spin_thread = threading.Thread(target=self.exec.spin, daemon=True)
        self.spin_thread.start()

        # UI
        self.widget = QWidget()
        self.widget.setObjectName('StingrayEngineerWidget')
        layout = QVBoxLayout(self.widget)

        # Top status labels
        self.lbl_depth = QLabel("Depth: N/A")
        self.lbl_depth.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.lbl_depth.setStyleSheet("font-weight:600;")
        layout.addWidget(self.lbl_depth)

        # Impact values labels
        self.impact_labels = {}
        impact_row = QHBoxLayout()
        for name in ('surge','sway','depth','roll','pitch','yaw'):
            lbl = QLabel(f"{name}: N/A")
            lbl.setMinimumWidth(80)
            impact_row.addWidget(lbl)
            self.impact_labels[name] = lbl
        layout.addLayout(impact_row)

        # Flags (checkboxes)
        flags_row = QHBoxLayout()
        self.flag_boxes = {}
        flag_names = [
            ('surge','Surge (bit0)'),
            ('sway','Sway (bit1)'),
            ('heave','Heave (bit2)'),
            ('yaw','Yaw (bit3)'),
            ('pitch','Pitch (bit4)'),
            ('roll','Roll (bit5)')
        ]
        for i,(key,label) in enumerate(flag_names):
            cb = QCheckBox(label)
            cb.setEnabled(False)
            flags_row.addWidget(cb)
            self.flag_boxes[i] = cb
        layout.addLayout(flags_row)

        # Thruster command list
        self.thruster_list = QListWidget()
        self.thruster_list.setMaximumHeight(120)
        layout.addWidget(QLabel("Thruster commands (/thruster/cmd):"))
        layout.addWidget(self.thruster_list)

        # Controls: slider publish depth setpoint (optional), start/stop publish
        ctrl_row = QHBoxLayout()
        self.depth_slider = QSlider(Qt.Horizontal)
        self.depth_slider.setRange(0, 500)  # 0..50.0 m
        self.depth_val_lbl = QLabel("0.0 m")
        ctrl_row.addWidget(QLabel("Depth set (m):"))
        ctrl_row.addWidget(self.depth_slider)
        ctrl_row.addWidget(self.depth_val_lbl)
        layout.addLayout(ctrl_row)

        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("Start publishing set")
        self.btn_stop = QPushButton("Stop publishing")
        self.btn_save = QPushButton("Save CSV")
        btn_row.addWidget(self.btn_start); btn_row.addWidget(self.btn_stop); btn_row.addWidget(self.btn_save)
        layout.addLayout(btn_row)

        # Plot area (pyqtgraph)
        if HAVE_PYQTGRAPH:
            self.plot = pg.PlotWidget(title="Signals")
            self.plot.addLegend()
            self.cur_depth = self.plot.plot([], [], pen=pg.mkPen(width=2), name='depth')
            # impacts: use thinner pens
            self.cur_surge = self.plot.plot([], [], pen=pg.mkPen(width=1), name='surge')
            self.cur_sway = self.plot.plot([], [], pen=pg.mkPen(width=1), name='sway')
            self.cur_impact_depth = self.plot.plot([], [], pen=pg.mkPen(width=1), name='impact_depth')
            layout.addWidget(self.plot)
        else:
            layout.addWidget(QLabel("Install pyqtgraph for live plotting (pip install pyqtgraph)"))

        context.add_widget(self.widget)

        # Data buffers
        self.t0 = time.time()
        self.tbuf = deque(maxlen=2000)
        self.depth_buf = deque(maxlen=2000)
        self.surge_buf = deque(maxlen=2000)
        self.sway_buf = deque(maxlen=2000)
        self.impact_depth_buf = deque(maxlen=2000)

        # State
        self.publishing = False

        # Connections
        self.depth_slider.valueChanged.connect(self._on_depth_slider)
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_save.clicked.connect(self._on_save)

        # UI update timer
        self.ui_timer = QTimer()
        self.ui_timer.setInterval(100)  # 10 Hz update UI
        self.ui_timer.timeout.connect(self._update_ui)
        self.ui_timer.start()

    # UI handlers
    def _on_depth_slider(self, v):
        val = v / 10.0
        self.depth_val_lbl.setText(f"{val:.1f} m")
        if self.publishing:
            try:
                self.ros_node.publish_depth_set(val)
            except Exception as e:
                self.ros_node.get_logger().error(f"publish error: {e}")

    def _on_start(self):
        self.publishing = True
        # publish once immediately
        try:
            val = self.depth_slider.value()/10.0
            self.ros_node.publish_depth_set(val)
        except Exception as e:
            self.ros_node.get_logger().error(f"publish error: {e}")

    def _on_stop(self):
        self.publishing = False

    def _update_ui(self):
        # read latest from ros_node
        d = self.ros_node.latest_depth
        imp = self.ros_node.latest_impact
        flags = self.ros_node.latest_flags
        thr = self.ros_node.latest_thruster_cmd
        t = time.time() - self.t0

        if d is not None:
            self.lbl_depth.setText(f"Depth: {d:.2f} m")
            self.tbuf.append(t)
            self.depth_buf.append(d)
        if imp['surge'] is not None:
            self.impact_labels['surge'].setText(f"surge: {imp['surge']:.2f}")
            self.surge_buf.append(imp['surge'])
        if imp['sway'] is not None:
            self.impact_labels['sway'].setText(f"sway: {imp['sway']:.2f}")
            self.sway_buf.append(imp['sway'])
        if imp['depth'] is not None:
            self.impact_labels['depth'].setText(f"depth: {imp['depth']:.2f}")
            self.impact_depth_buf.append(imp['depth'])
        for i in range(6):
            checked = bool(flags & (1 << i))
            cb = self.flag_boxes.get(i)
            if cb:
                cb.setChecked(checked)

        # thruster list
        if thr:
            self.thruster_list.clear()
            for i, val in enumerate(thr):
                item = QListWidgetItem(f"thr[{i}] = {val}")
                self.thruster_list.addItem(item)

        # update plots
        if HAVE_PYQTGRAPH and len(self.tbuf):
            times = list(self.tbuf)
            self.cur_depth.setData(times, list(self.depth_buf))
            # impacts might have different lengths; use tbuf for x axis
            # pad arrays to len(tbuf) if necessary (simple approach: use last-known values)
            def pad_buf(buf):
                if len(buf) == 0:
                    return [0]*len(times)
                if len(buf) < len(times):
                    # extend by repeating last value
                    return list(buf) + [buf[-1]]*(len(times)-len(buf))
                return list(buf[-len(times):])
            self.cur_surge.setData(times, pad_buf(self.surge_buf))
            self.cur_sway.setData(times, pad_buf(self.sway_buf))
            self.cur_impact_depth.setData(times, pad_buf(self.impact_depth_buf))

    def _on_save(self):
        fn, _ = QFileDialog.getSaveFileName(self.widget, "Save CSV", "", "CSV Files (*.csv)")
        if not fn:
            return
        # save t, depth, surge, sway, impact_depth
        with open(fn, 'w') as f:
            f.write('t,depth,surge,sway,impact_depth\n')
            n = max(len(self.tbuf), len(self.depth_buf), len(self.surge_buf), len(self.sway_buf), len(self.impact_depth_buf))
            for i in range(n):
                t = self.tbuf[i] if i < len(self.tbuf) else ''
                d = self.depth_buf[i] if i < len(self.depth_buf) else ''
                sg = self.surge_buf[i] if i < len(self.surge_buf) else ''
                sw = self.sway_buf[i] if i < len(self.sway_buf) else ''
                idp = self.impact_depth_buf[i] if i < len(self.impact_depth_buf) else ''
                f.write(f"{t},{d},{sg},{sw},{idp}\n")

    def shutdown_plugin(self):
        try:
            self.ui_timer.stop()
            self.exec.shutdown()
            self.exec.remove_node(self.ros_node)
            self.ros_node.destroy_node()
            if self._rclpy_inited_here:
                rclpy.shutdown()
        except Exception:
            pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
