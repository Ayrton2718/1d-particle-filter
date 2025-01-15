import rclpy
from rclpy.node import Node

import math
import os
import msgpack
import json
import svgelements
import cv2
import numpy as np

OPENCV_DEBUG_ON = False


class LcConverter(Node):
    """
    送信側
    """
    # ノード名
    NODE_NAME = "lc_converter"

    OBS_TYPES = [
        ["area1_wall",  0xFF9933],
        ["area1_hill",  0xFF9934],
        ["area1_half",  0xFF9935],
        ["area2_wall",  0x2D7600],
        ["area2_hill",  0x2D7601],
        ["area23_wall", 0x80FF00],
        ["area3_wall",  0xFFFF00],
        ["silo_wall",  0xFFFF01],
        ["field", 0x001DBC],
        ["seedling", 0x001DBD],
        ["plant", 0x001DBE],
        ["silo", 0x001DBF],
        ["ignore", 0x808080]
    ]

    OBS_LINES = [([]) for _ in OBS_TYPES]
    OBS_CIRCLES = [([]) for _ in OBS_TYPES]

    OFFSET = (6.05, 0)

    def __init__(self):
        """
        コンストラクタ
        Parameters
        ----------
        """
        # ノードの初期化
        super().__init__(self.NODE_NAME)

        self.get_logger().info("%s start..." % (self.NODE_NAME))
        
        self.declare_parameter('ws_dir', 'none')
        self.declare_parameter('map_dir', 'none')
        self.declare_parameter('map_name', 'none')
        ws_dir = self.get_parameter('ws_dir').get_parameter_value().string_value
        map_dir = self.get_parameter('map_dir').get_parameter_value().string_value
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        if map_dir == 'none' or map_name == 'none':
            self.get_logger().error("Failed to get params")
        self.get_logger().info('Hello ' + str(ws_dir) + ', ' + str(map_dir) + ', ' + str(map_name))

        map_dir = ws_dir + "/" + map_dir + map_name + "/"

        # 最大座標と最小座標の走査
        raw_max_y = -math.inf
        points = []
        circles = []
        # mapの読み込み
        for svg_obj in svgelements.SVG.parse(map_dir + "map.drawio.svg")[0]:
            if(type(svg_obj) == svgelements.svgelements.Path):
                assign_max_f = lambda x, y: x if(y < x) else y

                stroke_width = svg_obj.stroke_width / 2
                x0 = (svg_obj.first_point.x - stroke_width) / 1000.0
                y0 = (svg_obj.first_point.y - stroke_width) / 1000.0
                x1 = (svg_obj.current_point.x - stroke_width) / 1000.0
                y1 = (svg_obj.current_point.y - stroke_width) / 1000.0
                points.append((x0, y0, x1, y1, svg_obj.stroke))

                raw_max_y = assign_max_f(y0, raw_max_y)
                raw_max_y = assign_max_f(y1, raw_max_y)
                pass
            elif(type(svg_obj) == svgelements.svgelements.Ellipse):
                stroke_width = svg_obj.stroke_width / 2
                x0 = (svg_obj.cx - stroke_width) / 1000.0
                y0 = (svg_obj.cy - stroke_width) / 1000.0
                rx = svg_obj.rx / 1000.0
                ry = svg_obj.rx / 1000.0
                circles.append((x0, y0, rx, ry, svg_obj.stroke))
                pass

        self.get_logger().info(str(circles))

        svg_offset = (self.OFFSET[0], raw_max_y - self.OFFSET[1])

        # 最大座標と最小座標の走査
        max_x = -math.inf
        max_y = -math.inf
        min_x = math.inf
        min_y = math.inf
        # Json用の配列作製
        for point in points:
            assign_max_f = lambda x, y: x if(y < x) else y
            assign_min_f = lambda x, y: y if(y < x) else x

            x0 = -(point[1] - svg_offset[1])
            y0 = -(point[0] - svg_offset[0])
            x1 = -(point[3] - svg_offset[1])
            y1 = -(point[2] - svg_offset[0])

            for type_i, type_info in enumerate(self.OBS_TYPES):
                if point[4] == svgelements.Color(type_info[1]):
                    max_x = assign_max_f(x0, max_x)
                    min_x = assign_min_f(x0, min_x)
                    max_y = assign_max_f(y0, max_y)
                    min_y = assign_min_f(y0, min_y)

                    max_x = assign_max_f(x1, max_x)
                    min_x = assign_min_f(x1, min_x)
                    max_y = assign_max_f(y1, max_y)
                    min_y = assign_min_f(y1, min_y)
                    self.OBS_LINES[type_i].append([x0, y0, x1 - x0, y1 - y0])
                    break

        self.get_logger().info(str(self.OBS_LINES))

        # Json用の配列作製
        for circle in circles:
            assign_max_f = lambda x, y: x if(y < x) else y
            assign_min_f = lambda x, y: y if(y < x) else x

            # TODO Is this center?
            x0 = -(circle[1] - svg_offset[1])
            y0 = -(circle[0] - svg_offset[0])
            rx = (circle[3])
            ry = (circle[2])

            for type_i, type_info in enumerate(self.OBS_TYPES):
                if circle[4] == svgelements.Color(type_info[1]):
                    max_x = assign_max_f(x0, max_x)
                    min_x = assign_min_f(x0, min_x)
                    max_y = assign_max_f(y0, max_y)
                    min_y = assign_min_f(y0, min_y)
                    self.OBS_CIRCLES[type_i].append([x0, y0, rx, ry])
                    break
            
        self.get_logger().info(str(self.OBS_CIRCLES))


        cnv_pos = lambda x, y: (int((x - min_x) * 100) + 2, int((y - min_y) * 100) + 2)
        img_size = cnv_pos(max_x + 2, max_y + 2)
        image = np.zeros((img_size[1], img_size[0], 3))
        
        for type_i, lines in enumerate(self.OBS_LINES):
            for line in lines:
                color = svgelements.Color(self.OBS_TYPES[type_i][1])
                cv2.line(image, cnv_pos(line[0], line[1]), cnv_pos(line[2]+line[0], line[3]+line[1]), 
                            (color.blue, color.green, color.red), 1)

        for type_i, circles in enumerate(self.OBS_CIRCLES):
            for circle in circles:
                color = svgelements.Color(self.OBS_TYPES[type_i][1])
                cv2.circle(image, cnv_pos(circle[0], circle[1]), int(circle[2]*100), (color.blue, color.green, color.red), 1)
    
        if OPENCV_DEBUG_ON:            
            cv2.namedWindow("input", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("input", 900, 1200) 
            cv2.imshow("input", image)
            cv2.waitKey(0)

        # map_file
        map_obj = {
            'max_pos': [max_x, max_y],
            'min_pos': [min_x, min_y],
            'obs_types': self.OBS_TYPES,
            'obs_lines': self.OBS_LINES,
            'obs_circles': self.OBS_CIRCLES,
        }

        export_path = map_dir + "map.mpac"

        obj1_bin = msgpack.packb(map_obj)
        self.get_logger().info("export line_map(" + str(len(obj1_bin)) + "[byte]) at " + export_path)
        with open(export_path, 'wb') as fd:
            fd.write(obj1_bin)

        with open(map_dir + 'map.json', 'w') as fd:
            json.dump(map_obj, fd, indent=2)
        
        raise SystemExit           # <--- here is we exit the node

    def __del__(self):
        """
        デストラクタ
        """
        # コンソールに表示
        self.get_logger().info("%s done." % self.NODE_NAME)


def main(args=None):
    """
    メイン関数
    Parameters
    ----------
    """
    try:
        # rclpyの初期化
        rclpy.init(args=args)

        # インスタンスを生成
        node = LcConverter()
        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        # 終了処理
        rclpy.shutdown()


if __name__ == '__main__':
    main()
