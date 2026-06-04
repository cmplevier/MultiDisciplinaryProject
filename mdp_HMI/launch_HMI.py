from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
    display = ":99"

    config_dir = os.path.dirname(os.path.abspath(__file__))
    pcl_rviz_config = os.path.join(config_dir, "pcl_visualization.rviz")
    image_rviz_config = os.path.join(config_dir, "image_visualization.rviz")
    map_rviz_config = os.path.join(config_dir, "rviz_map_visualization.rviz")

    screen_width = 1920
    pane_margin = 20
    pane_gap = 16
    title_height = 32
    pane_height = 480
    top_y = 20
    bottom_y = top_y + title_height + pane_height + pane_gap

    half_width = (screen_width - (2 * pane_margin) - pane_gap) // 2
    full_width = screen_width - (2 * pane_margin)
    pcl_x = pane_margin
    image_x = pane_margin + half_width + pane_gap
    map_x = pane_margin

    panes = [
        ("PCL_visualization", pcl_rviz_config, pcl_x, top_y, half_width, pane_height),
        ("image_visualization", image_rviz_config, image_x, top_y, half_width, pane_height),
        ("map_visualization", map_rviz_config, map_x, bottom_y, full_width, pane_height),
    ]

    def rviz_window(name, config, x, y, width, height):
        rviz_y = y + title_height
        return ExecuteProcess(
            cmd=["bash", "-lc", f"""
            export DISPLAY={display}
            export QT_QPA_PLATFORM=xcb
            export XDG_SESSION_TYPE=x11

            export QT_SCALE_FACTOR=0.5
            export QT_AUTO_SCREEN_SCALE_FACTOR=0
            export QT_ENABLE_HIGHDPI_SCALING=1

            rviz2 -d "{config}" &
            PID=$!

            sleep 3

            WIN=""
            for _ in $(seq 1 30); do
              WIN=$(xdotool search --onlyvisible --pid $PID 2>/dev/null | head -n 1)
              [ -n "$WIN" ] && break
              sleep 0.2
            done

            if [ -n "$WIN" ]; then
              xdotool windowmove $WIN {x} {rviz_y}
              xdotool windowsize $WIN {width} {height}
              xdotool set_window --name "{name}" $WIN
            else
              echo "Could not find the {name} RViz window to place it." >&2
            fi

            wait $PID
            """],
            output="screen",
        )

    xvfb = ExecuteProcess(
        cmd=["bash", "-lc", f"""
        rm -f /tmp/.X99-lock

        Xvfb {display} \
          -screen 0 1920x1080x24 \
          -ac +extension GLX +render -noreset
        """],
        output="screen",
    )

    title_bars = ExecuteProcess(
        cmd=["bash", "-lc", f"""
        export DISPLAY={display}

        python3 - <<'PY'
import tkinter as tk

PANES = {[(name, x, y, width, title_height) for name, _config, x, y, width, _height in panes]!r}

root = tk.Tk()
root.withdraw()

for title, x, y, width, height in PANES:
    bar = tk.Toplevel(root)
    bar.title(title)
    bar.overrideredirect(True)
    bar.geometry(f"{{width}}x{{height}}+{{x}}+{{y}}")
    bar.configure(bg="#151923", highlightbackground="#6b7280", highlightthickness=1)

    label = tk.Label(
        bar,
        text=title,
        bg="#151923",
        fg="#f9fafb",
        font=("DejaVu Sans", 14, "bold"),
        anchor="center",
        padx=12,
    )
    label.pack(fill="both", expand=True)

root.mainloop()
PY
        """],
        output="screen",
    )

    pcl_rviz = rviz_window(*panes[0])
    image_rviz = rviz_window(*panes[1])
    map_rviz = rviz_window(*panes[2])

    vnc = ExecuteProcess(
        cmd=["bash", "-lc", f"""
        unset WAYLAND_DISPLAY
        unset XDG_CURRENT_DESKTOP

        export XDG_SESSION_TYPE=x11
        export DISPLAY={display}

        x11vnc \
          -display {display} \
          -nopw \
          -localhost \
          -forever \
          -shared \
          -rfbport 5900 \
          -xkb \
          -noxdamage
        """],
        output="screen",
    )

    return LaunchDescription([
        xvfb,
        TimerAction(period=1.0, actions=[title_bars]),
        TimerAction(period=2.0, actions=[pcl_rviz]),
        TimerAction(period=5.0, actions=[image_rviz]),
        TimerAction(period=8.0, actions=[map_rviz]),
        TimerAction(period=11.0, actions=[vnc]),
    ])
