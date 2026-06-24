gnome-terminal --window -e 'bash -c "exec bash"' \
--tab -e 'bash -c "sleep 1; source ~/shoot_race/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; cd reference/; source ~/shoot_race/devel/setup.bash; python3 shoot_race_shoot1_only.py; exec bash"' \










