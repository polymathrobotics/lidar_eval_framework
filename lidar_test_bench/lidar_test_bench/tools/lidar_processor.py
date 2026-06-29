import numpy as np
import struct
from typing import Sequence

class LidarProcessor:
    def __init__(self):
        self.points_array = []
        self.x_byte_offset = 0
        self.x_data_type = 0
        self.y_byte_offset = 0
        self.y_data_type = 0
        self.z_byte_offset = 0
        self.z_data_type = 0
        self.intensity_byte_offset = 0
        self.intensity_data_type = 0
        self.has_intensity = False

    def convert_pointcloud2_to_points(self, pointcloud2_msg):
        # Reset local array for the new frame
        self.points_array = []
        self.has_intensity = False
        num_of_points = pointcloud2_msg.height * pointcloud2_msg.width
        for field in pointcloud2_msg.fields:
            if field.name == "x":
                self.x_byte_offset = field.offset
                self.x_data_type = field.datatype
            elif field.name == "y":
                self.y_byte_offset = field.offset
                self.y_data_type = field.datatype
            elif field.name == "z":
                self.z_byte_offset = field.offset
                self.z_data_type = field.datatype
            elif field.name == "intensity":
                self.intensity_byte_offset = field.offset
                self.intensity_data_type = field.datatype
                self.has_intensity = True

        if self.z_data_type != self.x_data_type or self.z_data_type != self.y_data_type or self.y_data_type != self.x_data_type:
            raise ValueError("x, y, z values have been corrupted")

        for i in range(num_of_points):
            byte_start = i * pointcloud2_msg.point_step
            x_coordinate = self.convert_bytes_to_coordinate(self.x_data_type, self.x_byte_offset, pointcloud2_msg.is_bigendian, byte_start, pointcloud2_msg.data)
            y_coordinate = self.convert_bytes_to_coordinate(self.y_data_type, self.y_byte_offset, pointcloud2_msg.is_bigendian, byte_start, pointcloud2_msg.data)
            z_coordinate = self.convert_bytes_to_coordinate(self.z_data_type, self.z_byte_offset, pointcloud2_msg.is_bigendian, byte_start, pointcloud2_msg.data)
            if self.has_intensity:
                intensity = self.convert_bytes_to_coordinate(self.intensity_data_type, self.intensity_byte_offset, pointcloud2_msg.is_bigendian, byte_start, pointcloud2_msg.data)
                self.points_array.append([x_coordinate, y_coordinate, z_coordinate, intensity])
            else:
                self.points_array.append([x_coordinate, y_coordinate, z_coordinate, 0.0])

        points_array = np.asarray(self.points_array, dtype=np.float32)
        return points_array, num_of_points

    def bytes_to_float32(self, is_bigendian: bool, b4_list: Sequence[int]) -> float:
        if len(b4_list) != 4:
            raise ValueError(f"Expected 4 byte-values, got {len(b4_list)}")
        b4 = bytes(b4_list)
        fmt = ">f" if is_bigendian else "<f"
        return struct.unpack(fmt, b4)[0]

    def bytes_to_float64(self, is_bigendian: bool, b8_list: Sequence[int]) -> float:
        return 1.0

    def convert_bytes_to_coordinate(self, datatype, byte_offset, is_bigendian, byte_start, data):
        match datatype:
            case 1 | 2:
                return 1
            case 3 | 4:
                return 2
            case 5 | 6 | 7:
                return self.bytes_to_float32(is_bigendian, data[byte_start + byte_offset: byte_start + byte_offset + 4])
            case 8:
                return self.bytes_to_float64(is_bigendian, data[byte_start + byte_offset: byte_start + byte_offset + 8])
            case _:
                raise ValueError(f"Unsupported PointField datatype: {datatype}")
