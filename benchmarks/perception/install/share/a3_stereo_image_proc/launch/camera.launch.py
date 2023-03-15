import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([


    Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='left_camera',
        namespace='left',
        output='screen',
        remappings=[('image_raw', 'left/image_raw')],
        parameters=[{'image_size': [640,480], 
                    'camera_frame_id': 'left_camera_link_optical'}]
    ),
    Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='right_camera',
        namespace='right',
        output='screen',
        remappings=[('image_raw', 'right/image_raw')],
        parameters=[{'image_size': [640,480],
                    'camera_frame_id': 'right_camera_link_optical'}]
    )
    ])
        

#     return LaunchDescription([
#         Node(
#             package='v4l2_camera',
#             executable='v4l2_camera_node',
#             output='screen',
#             parameters=[{
#                 'image_size': [640,480],
#                 'camera_frame_id': 'left_camera_optical_frame'
#                 }]
#     )
# ])




# Node(
#     package='v4l2_camera',
#     executable='v4l2_camera_node',
#     name='left_camera',
#     namespace='left',
#     output='screen',
#     remappings=[('image_raw', 'left/image_raw')],
#     parameters=[{'image_size': [640,480], 
#                 'camera_frame_id': 'left_camera_link_optical'}]
# ),
# Node(
#     package='v4l2_camera',
#     executable='v4l2_camera_node',
#     name='right_camera',
#     namespace='right',
#     output='screen',
#     remappings=[('image_raw', 'right/image_raw')],
#     parameters=[{'image_size': [640,480],
#                 'camera_frame_id': 'right_camera_link_optical'}]
# )

# return LaunchDescription([

#     Node(
#         package='v4l2_camera',
#         executable='v4l2_camera_node',
#         output='screen',
#         parameters=[{
#             'image_size': [640,480],
#             'camera_frame_id': 'camera_link_optical_right'
#             }]
# )
# ])



# Node(
#     package='v4l2_camera',
#     executable='v4l2_camera_node',
#     name='left_camera',
#     namespace='left',
#     output='screen',
#     parameters=[{'image_size': [640,480], 
#                  'camera_frame_id': 'left_camera_link_optical'}]
# ),
# Node(
#     package='v4l2_camera',
#     executable='v4l2_camera_node',
#     name='right_camera',
#     namespace='right',
#     output='screen',
#     parameters=[{'image_size': [640,480],
#                  'camera_frame_id': 'right_camera_link_optical'}]
# )