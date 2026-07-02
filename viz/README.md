# viz/ — Visualizzazione RViz del robot (lato controller)

Copia di **deploy** del modello dell'esapode, per visualizzarlo in **RViz** sul
display 7" del controller (console operatore). Solo file statici: RViz non ha
bisogno del generatore.

⚠️ **Fonte di verità = repo `RobotHex`**, cartella `description/` (script
`gen_urdf.py`, che genera l'URDF da `leg_config.py`). Questi file qui sono
l'**output** copiato per l'uso. Se il modello cambia: si rigenera in RobotHex e
si ricopiano `genghis.urdf` / `display.launch.py` / `genghis.rviz` qui.

## File
- `genghis.urdf` — modello (22 link, 21 giunti)
- `display.launch.py` — avvia robot_state_publisher + joint_state_publisher_gui + rviz2
- `genghis.rviz` — config RViz (RobotModel + TF + griglia)

## Uso sul controller (Ubuntu 22.04 + ROS2 Humble)

Dipendenze (una volta):
```bash
sudo apt install -y ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

Avvio (finestra sul display 7"):
```bash
export DISPLAY=:0        # se lanci da SSH; non serve se sei sul terminale del 7"
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/viz
ros2 launch ./display.launch.py
```

Si aprono RViz + una finestra con uno slider per ogni giunto. A zero le gambe
sono orizzontali (posa di calibrazione): alza i `*_lift` per vedere il robot "in
piedi". Se RViz ha problemi OpenGL sul Pi: `export LIBGL_ALWAYS_SOFTWARE=1`.
