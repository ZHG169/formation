formation package 逐檔程式碼說明索引

這個資料夾是一份「每個程式碼/設定/介面檔各自一份說明」的閱讀輔助。
排除項目：log、__pycache__、build/install 產物。

建議閱讀順序：
1. config/formation.yaml
2. formation/main_node.py
3. formation/mission_manager.py
4. formation/vehicle_interface.py
5. formation/formation_controller.py
6. formation/formation_shapes.py
7. formation/cpf_avoidance.py
8. formation/safety_manager.py
9. leader/distributed/launch/msg/srv 相關檔案

檔案對照：
- CMakeLists.txt -> CMakeLists.txt.explanation.txt
- package.xml -> package.xml.explanation.txt
- config/formation.yaml -> config_formation.yaml.explanation.txt
- formation/__init__.py -> formation___init__.py.explanation.txt
- formation/coordinate_convert.py -> formation_coordinate_convert.py.explanation.txt
- formation/cpf_avoidance.py -> formation_cpf_avoidance.py.explanation.txt
- formation/distributed_vehicle_node.py -> formation_distributed_vehicle_node.py.explanation.txt
- formation/formation_controller.py -> formation_formation_controller.py.explanation.txt
- formation/formation_shapes.py -> formation_formation_shapes.py.explanation.txt
- formation/leader_command_node.py -> formation_leader_command_node.py.explanation.txt
- formation/leader_manager.py -> formation_leader_manager.py.explanation.txt
- formation/main_node.py -> formation_main_node.py.explanation.txt
- formation/mission_manager.py -> formation_mission_manager.py.explanation.txt
- formation/safety_manager.py -> formation_safety_manager.py.explanation.txt
- formation/vehicle_interface.py -> formation_vehicle_interface.py.explanation.txt
- launch/formation.launch.py -> launch_formation.launch.py.explanation.txt
- launch/multi_PX4.launch.py -> launch_multi_PX4.launch.py.explanation.txt
- msg/FormationCommand.msg -> msg_FormationCommand.msg.explanation.txt
- msg/FormationError.msg -> msg_FormationError.msg.explanation.txt
- msg/FormationStatus.msg -> msg_FormationStatus.msg.explanation.txt
- scripts/distributed_vehicle_node -> scripts_distributed_vehicle_node.explanation.txt
- scripts/formation_node -> scripts_formation_node.explanation.txt
- scripts/leader_command_node -> scripts_leader_command_node.explanation.txt
- scripts/monitor_mav_dds.sh -> scripts_monitor_mav_dds.sh.explanation.txt
- src/multi_takeoff_land.py -> src_multi_takeoff_land.py.explanation.txt
- srv/SetFormation.srv -> srv_SetFormation.srv.explanation.txt
- srv/SetLeader.srv -> srv_SetLeader.srv.explanation.txt
