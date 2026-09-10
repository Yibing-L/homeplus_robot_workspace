#!/usr/bin/env python3

# Copyright 2026 Yibing Lu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Live MolmoPoint test UI backed by the production ROS target node."""

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
from cv_bridge import CvBridge
from PIL import Image as PILImage
from PIL import ImageTk
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class MolmoPointTestClient(Node):
    """ROS-facing half of the UI; it never performs or reimplements inference."""

    def __init__(self):
        super().__init__('molmopoint_test_interface')
        self.declare_parameter('detector_node', '/molmopoint')
        self.declare_parameter('debug_image_topic', '/molmopoint/debug_image')
        self.declare_parameter('detections_topic', '/molmopoint/detections')
        self.declare_parameter('initial_caption', 'the coffee cup')

        detector_node = str(self.get_parameter('detector_node').value).rstrip('/')
        if not detector_node.startswith('/'):
            detector_node = '/' + detector_node
        debug_topic = str(self.get_parameter('debug_image_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)

        self.initial_caption = str(
            self.get_parameter('initial_caption').value
        )
        self.bridge = CvBridge()
        self.events = queue.Queue()
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._last_frame_time = None
        self._display_rate = 0.0
        self._have_frame = False
        self._parameter_lock = threading.Lock()
        self._pending_controls = None
        self._parameter_future = None

        self.create_subscription(
            Image,
            debug_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            detections_topic,
            self._detections_callback,
            10,
        )
        self.parameter_client = self.create_client(
            SetParameters,
            f'{detector_node}/set_parameters',
        )
        self.detector_node = detector_node
        self.create_timer(0.25, self._send_pending_controls)

    def _image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:
            self.events.put(('status', f'Could not decode debug image: {exc}', True))
            return

        now = time.monotonic()
        if self._last_frame_time is not None:
            elapsed = now - self._last_frame_time
            if elapsed > 0.0:
                instantaneous_rate = 1.0 / elapsed
                self._display_rate = (
                    instantaneous_rate
                    if self._display_rate == 0.0
                    else 0.8 * self._display_rate + 0.2 * instantaneous_rate
                )
        self._last_frame_time = now
        with self._frame_lock:
            self._latest_frame = (frame, self._display_rate)
        if not self._have_frame:
            self._have_frame = True
            self.events.put(('status', 'Receiving production detector output.', False))

    def take_latest_frame(self):
        with self._frame_lock:
            frame = self._latest_frame
            self._latest_frame = None
        return frame

    def _detections_callback(self, message):
        try:
            detections = json.loads(message.data)
            if not isinstance(detections, list):
                raise ValueError('payload is not a list')
            self.events.put(('detections', detections))
        except (json.JSONDecodeError, ValueError) as exc:
            self.events.put(('status', f'Invalid detection payload: {exc}', True))

    def apply_controls(self, caption):
        caption = caption.strip()
        if not caption:
            self.events.put(('status', 'Enter at least one keyword.', True))
            return
        with self._parameter_lock:
            self._pending_controls = caption
        self._send_pending_controls()

    def _send_pending_controls(self):
        with self._parameter_lock:
            if self._pending_controls is None or self._parameter_future is not None:
                return
            if not self.parameter_client.service_is_ready():
                self.events.put((
                    'status',
                    f'Waiting for detector service on {self.detector_node}…',
                    False,
                ))
                return
            caption = self._pending_controls
            self._pending_controls = None
            # Reserve the in-flight slot before releasing the lock so a timer
            # and a button click cannot issue overlapping service requests.
            self._parameter_future = True

        request = SetParameters.Request()
        request.parameters = [
            Parameter('caption', value=caption).to_parameter_msg(),
        ]
        try:
            future = self.parameter_client.call_async(request)
        except Exception as exc:
            with self._parameter_lock:
                self._parameter_future = None
            self.events.put(('status', f'Parameter update failed: {exc}', True))
            return
        with self._parameter_lock:
            self._parameter_future = future
        future.add_done_callback(self._parameters_applied)
        self.events.put(('status', 'Applying detection controls…', False))

    def _parameters_applied(self, future):
        with self._parameter_lock:
            self._parameter_future = None
        try:
            response = future.result()
            failures = [result.reason for result in response.results if not result.successful]
            if failures:
                self.events.put(('status', '; '.join(failures), True))
            else:
                self.events.put(('status', 'Live controls applied.', False))
        except Exception as exc:
            self.events.put(('status', f'Parameter update failed: {exc}', True))
        self._send_pending_controls()


class MolmoPointTestWindow:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title('HomePlus · MolmoPoint Test')
        self.root.geometry('1180x760')
        self.root.minsize(920, 620)
        self.root.configure(bg='#10151d')
        self._photo = None
        self._closing = False

        self._configure_style()
        self._build_layout()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(40, self._poll)

    def _configure_style(self):
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', background='#10151d', foreground='#e8eef7')
        style.configure('Panel.TFrame', background='#18212c')
        style.configure(
            'Title.TLabel',
            background='#10151d',
            foreground='#f7f9fc',
            font=('TkDefaultFont', 18, 'bold'),
        )
        style.configure(
            'Muted.TLabel',
            background='#18212c',
            foreground='#9aabba',
        )
        style.configure(
            'Status.TLabel',
            background='#18212c',
            foreground='#77d7b5',
            padding=(0, 8),
        )
        style.configure(
            'Error.Status.TLabel',
            background='#18212c',
            foreground='#ff8a8a',
            padding=(0, 8),
        )
        style.configure(
            'Accent.TButton',
            background='#2f80ed',
            foreground='white',
            padding=(14, 8),
            font=('TkDefaultFont', 10, 'bold'),
        )
        style.map('Accent.TButton', background=[('active', '#4994f2')])
        style.configure(
            'Treeview',
            background='#111923',
            fieldbackground='#111923',
            foreground='#e8eef7',
            rowheight=27,
            borderwidth=0,
        )
        style.configure(
            'Treeview.Heading',
            background='#222e3c',
            foreground='#aebdca',
            relief='flat',
        )

    def _build_layout(self):
        header = ttk.Frame(self.root, padding=(22, 18, 22, 12))
        header.pack(fill='x')
        ttk.Label(header, text='MolmoPoint · Live Test', style='Title.TLabel').pack(
            side='left'
        )
        self.rate_var = tk.StringVar(value='waiting for debug frames')
        ttk.Label(
            header,
            textvariable=self.rate_var,
            background='#10151d',
            foreground='#8ea3b6',
        ).pack(side='right', pady=(8, 0))

        content = ttk.Frame(self.root, padding=(22, 0, 22, 22))
        content.pack(fill='both', expand=True)
        content.columnconfigure(0, weight=4)
        content.columnconfigure(1, weight=2)
        content.rowconfigure(0, weight=1)

        viewer = tk.Frame(
            content,
            bg='#070a0f',
            highlightthickness=1,
            highlightbackground='#293544',
        )
        viewer.grid(row=0, column=0, sticky='nsew', padx=(0, 14))
        self.image_label = tk.Label(
            viewer,
            text='Waiting for /molmopoint/debug_image…',
            bg='#070a0f',
            fg='#75889a',
            font=('TkDefaultFont', 12),
        )
        self.image_label.pack(fill='both', expand=True)

        panel = ttk.Frame(content, style='Panel.TFrame', padding=18)
        panel.grid(row=0, column=1, sticky='nsew')
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(6, weight=1)

        ttk.Label(panel, text='PROMPT', style='Muted.TLabel').grid(
            row=0, column=0, sticky='w'
        )
        self.caption_var = tk.StringVar(value=self.node.initial_caption)
        caption_entry = tk.Entry(
            panel,
            textvariable=self.caption_var,
            bg='#0f1721',
            fg='#f2f5f8',
            insertbackground='white',
            relief='flat',
            font=('TkDefaultFont', 12),
        )
        caption_entry.grid(row=1, column=0, sticky='ew', pady=(7, 4), ipady=9)
        caption_entry.bind('<Return>', lambda _event: self._apply())
        ttk.Label(
            panel,
            text='Use a natural-language target, e.g. “the red mug on the left”.',
            style='Muted.TLabel',
            wraplength=290,
        ).grid(row=2, column=0, sticky='w', pady=(0, 18))

        ttk.Button(
            panel,
            text='Apply live',
            command=self._apply,
            style='Accent.TButton',
        ).grid(row=3, column=0, sticky='ew')

        self.status_var = tk.StringVar(
            value=f'Connecting to {self.node.detector_node}…'
        )
        self.status_label = ttk.Label(
            panel,
            textvariable=self.status_var,
            style='Status.TLabel',
            wraplength=290,
        )
        self.status_label.grid(row=4, column=0, sticky='ew')

        ttk.Separator(panel, orient='horizontal').grid(
            row=5, column=0, sticky='ew', pady=(7, 14)
        )
        self.detection_summary = tk.StringVar(value='DETECTIONS · 0')
        ttk.Label(
            panel,
            textvariable=self.detection_summary,
            style='Muted.TLabel',
        ).grid(row=5, column=0, sticky='w', pady=(22, 7))

        self.table = ttk.Treeview(
            panel,
            columns=('label', 'point'),
            show='headings',
            selectmode='none',
        )
        self.table.heading('label', text='Label')
        self.table.heading('point', text='Point (u, v)')
        self.table.column('label', width=150, stretch=True)
        self.table.column('point', width=105, anchor='center', stretch=True)
        self.table.grid(row=6, column=0, sticky='nsew')

    def _apply(self):
        self.node.apply_controls(self.caption_var.get())

    def _poll(self):
        if self._closing:
            return
        if not rclpy.ok():
            self.close()
            return

        frame_data = self.node.take_latest_frame()
        if frame_data is not None:
            self._show_frame(*frame_data)

        while True:
            try:
                event = self.node.events.get_nowait()
            except queue.Empty:
                break
            if event[0] == 'detections':
                self._show_detections(event[1])
            elif event[0] == 'status':
                self.status_var.set(event[1])
                self.status_label.configure(
                    style='Error.Status.TLabel' if event[2] else 'Status.TLabel'
                )
        self.root.after(40, self._poll)

    def _show_frame(self, frame, rate):
        available_width = max(320, self.image_label.winfo_width() - 16)
        available_height = max(240, self.image_label.winfo_height() - 16)
        height, width = frame.shape[:2]
        scale = min(available_width / width, available_height / height)
        display_size = (max(1, int(width * scale)), max(1, int(height * scale)))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = PILImage.fromarray(rgb).resize(display_size, PILImage.Resampling.LANCZOS)
        self._photo = ImageTk.PhotoImage(image=image)
        self.image_label.configure(image=self._photo, text='')
        self.rate_var.set(f'{rate:.2f} processed frames/s')

    def _show_detections(self, detections):
        self.table.delete(*self.table.get_children())
        for detection in detections:
            point = detection.get('point', [])
            point_text = ', '.join(str(int(value)) for value in point) if point else '—'
            self.table.insert(
                '',
                'end',
                values=(
                    detection.get('label', 'unknown'),
                    point_text,
                ),
            )
        self.detection_summary.set(f'DETECTIONS · {len(detections)}')

    def run(self):
        self.root.mainloop()

    def close(self):
        if self._closing:
            return
        self._closing = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MolmoPointTestClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        window = MolmoPointTestWindow(node)
        window.run()
    except tk.TclError as exc:
        node.get_logger().error(f'Could not open the test window: {exc}')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
