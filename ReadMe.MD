## Turtle Bot


## Connecting to be bot via SSH

- Ensure bot is switched on
- Use the following code in the terminal

```bash
    ssh robotname
```
~ robotname is the name of your robot i.e. cseeburger01

- Enter your password for fingerprint signing

## Turtle Bot Command

Run the below command to start the bot, a succesfull start would show RUN!

```bash
    ros2 launch turtlebot3_bringup robot.launch.py
```

## Running your code on the bot 

- To do this open a new terminal and and repeat the connection process via ssh
- Once connected, navigate to your script folder
- Then you can run your python code

```bash
    python3 codename.py
```
