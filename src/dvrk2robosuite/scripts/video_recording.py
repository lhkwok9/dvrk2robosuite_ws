#!/usr/bin/env python
"""
Record video of agent episodes with the imageio library.
This script uses offscreen rendering.

Example:
    $ python demo_video_recording.py --environment Lift --robots Panda
"""

import argparse
import os

import imageio
import numpy as np

import robosuite.macros as macros
from robosuite import make
from robosuite.renderers import load_renderer_config

# Set the image convention to opencv so that the images are automatically rendered "right side up" when using imageio
# (which uses opencv convention)
macros.IMAGE_CONVENTION = "opencv"


def record_video(video_dir:str, camera_num:int, fps:int):
    """record videos
    """
    # create a video writer with imageio
    writer = []
    for i in range(camera_num):
        os.makedirs(os.path.join( video_dir, f'{i}'), exist_ok=True)
        writer.append(imageio.get_writer("./images/" + f'{i}' + ".mp4", fps=fps))
    

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", type=str, default="TwoArmLift")
    parser.add_argument("--robots", nargs="+", type=str, default=["Panda", "Panda"], help="Which robot(s) to use in the env")
    parser.add_argument("--timesteps", type=int, default=300)
    parser.add_argument("--camera", nargs="+", type=str, default=["customfrontview", "custombirdview", "robot0_eye_in_hand", "robot1_eye_in_hand"])
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--skip_frame", type=int, default=1)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    # initialize an environment with offscreen renderer
    env = make(
        args.environment,
        args.robots,
        has_renderer=False,
        ignore_done=True,
        use_camera_obs=True,
        use_object_obs=False,
        camera_names=args.camera,
        camera_heights=args.height,
        camera_widths=args.width,
        # renderer="nvisii",
        # renderer_config=load_renderer_config("nvisii"),
        # camera_segmentations=None,
    )

    obs = env.reset()
    ndim = env.action_dim

    # create a video writer with imageio
    writer = []
    camera_num = len(args.camera)
    print(camera_num)
    try: 
        os.mkdir("./images")
        for i in range(camera_num):
            os.mkdir("./images/" + args.camera[i])
            writer.append(imageio.get_writer("./images/" + args.camera[i] + ".mp4", fps=args.fps))
    except:
        pass

    for j in range(args.timesteps):

        # run a uniformly random agent
        action = 0.5 * np.random.randn(ndim)
        obs, reward, done, info = env.step(action)

        # dump a frame from every K frames
        if j % args.skip_frame == 0:
            frame = []
            for i in range(camera_num):
                frame.append(obs[args.camera[i] + "_image"])
                # print(frame.shape)
                writer[i].append_data(frame[i])
                with open(f'./images/{args.camera[i]}/{j}.npy', 'wb') as f:
                    np.save(f, frame[i])
                    f.close
            print("Saving frame #{}".format(j))

        if done:
            break

    for i in range(camera_num):
        writer[i].close()
