std_msgs/Header header

mavros_msgs/State state
mavros_msgs/EstimatorStatus estimator_status
mavros_msgs/ExtendedState extended_state
geometry_msgs/Vector3 attitude_angle
geometry_msgs/Vector3 attitude_rate
geometry_msgs/Vector3 local_position
geometry_msgs/Vector3 local_velocity
geometry_msgs/Vector3 body_heading_velocity #指机体坐标映射到的参考系的水平方向（即参考系到机体系，只旋转了Z轴）Head X+  Left Y+  Up Z+