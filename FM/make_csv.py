# csvの書き込み

import copy
import inspect
import sys
import time
import traceback

try:
    DEBUG = True

    msg_types = ['time', 'file', 'func', 'line', 'serious_error', 'error', 'warning', 'msg', 'format_exception', 'phase', 'gnss_time', 'date', 'lat', 'lon', 'alt', 'alt_base_press', 'goal_lat', 'goal_lon', 'temp', 'press', 'camera_area', 'camera_order', 'camera_center_x', 'camera_center_y', 'camera_frame_size_x', 'camera_frame_size_y', 'motor_l', 'motor_r', 'goal_relative_x', 'goal_relative_y', 'goal_relative_angle_rad', 'goal_distance', 'accel_all_x', 'accel_all_y', 'accel_all_z', 'accel_line_x', 'accel_line_y', 'accel_line_z', 'mag_x', 'mag_y', 'mag_z', 'gyro_x', 'gyro_y', 'gyro_z', 'grav_x', 'grav_y', 'grav_z', 'euler_x', 'euler_y', 'euler_z', 'nmea']
    DEFAULT_DICT = {x : '' for x in msg_types}

    filename = 'log.csv'

    with open(filename, 'a') as f:
        f.write('\n\n\n\n' + ','.join(msg_types) + '\n')
except Exception as e:
    print(f"An error occured in init csv: {e}")

def print(msg_type : str, msg_data):
    try:
        output_dict = copy.copy(DEFAULT_DICT)
        if (msg_type == 'accel_all') or (msg_type == 'accel_line') or (msg_type == 'mag') or (msg_type == 'gyro') or (msg_type == 'grav') or(msg_type == 'euler'):
            output_dict[msg_type + '_x'] = '"' + str(msg_data[0]).replace('"', '""') + '"'
            output_dict[msg_type + '_y'] = '"' + str(msg_data[1]).replace('"', '""') + '"'
            output_dict[msg_type + '_z'] = '"' + str(msg_data[2]).replace('"', '""') + '"'
        elif (msg_type == 'goal_relative') or (msg_type == 'camera_center') or (msg_type == 'camera_frame_size'):
            output_dict[msg_type + '_x'] = '"' + str(msg_data[0]).replace('"', '""') + '"'
            output_dict[msg_type + '_y'] = '"' + str(msg_data[1]).replace('"', '""') + '"'
        elif (msg_type == 'motor'):
            output_dict[msg_type + '_l'] = '"' + str(msg_data[0]).replace('"', '""') + '"'
            output_dict[msg_type + '_r'] = '"' + str(msg_data[1]).replace('"', '""') + '"'
        elif msg_type == 'lat_lon':
            output_dict['lat'] = '"' + str(msg_data[0]).replace('"', '""') + '"'
            output_dict['lon'] = '"' + str(msg_data[1]).replace('"', '""') + '"'
        else:
            output_dict[msg_type] = '"' + str(msg_data).replace('"', '""') + '"'

        output_dict['time'] = '"' + str(time.monotonic()) + '"'
        if DEBUG:
            try:
                frame = inspect.currentframe().f_back
                output_dict['file'] = '"' + str(frame.f_code.co_filename) + '"'
                output_dict['func'] = '"' + str(frame.f_code.co_name) + '"'
                output_dict['line'] = '"' + str(frame.f_lineno) + '"'
            except Exception as e:
                print(f"An error occured in inspecting fileinfo: {e}")
            
            try:
                e_type, e_obj, e_trace = sys.exc_info()
                if e_obj is not None:
                    f_exp = traceback.format_exception(e_type, e_obj, e_trace)
                    output_dict['format_exception'] = '"' + str(f_exp[0] + f_exp[1] + f_exp[2]).replace('"', '""') + '"'
            except Exception as e:
                print(f"An error occured in inspecting error_info: {e}")

        output_msg = ','.join(output_dict.values())
        # print(output_msg)
        with open(filename, 'a') as f:
            f.write(output_msg + '\n')
    except Exception as e:
        print(f"An error occured in printing to csv: {e}")
