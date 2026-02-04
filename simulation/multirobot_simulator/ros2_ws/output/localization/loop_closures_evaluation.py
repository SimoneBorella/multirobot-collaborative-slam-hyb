import argparse
import numpy as np
import matplotlib.pyplot as plt

def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate loop closures")
    parser.add_argument('--keyframes_filename', type=str, default="output/localization/keyframes.csv", help="Keyframes data")
    parser.add_argument('--loop_closures_filename', type=str, default="output/localization/loop_closures.csv", help="Loop closures data")
    return parser.parse_args()

def quat_to_yaw(qw, qx, qy, qz):
    return np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    )

def pose_to_T(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1]
    ])

def main():
    args = parse_args()

    # Read keyframes
    keyframes = {}
    xs, ys = [], []

    with open(args.keyframes_filename, 'r') as f:
        lines = f.readlines()

    for line in lines[1:]:
        tokens = line.strip().split(',')
        if len(tokens) < 10:
            continue

        kf_id = int(tokens[1])
        x = float(tokens[3])
        y = float(tokens[4])
        qw = float(tokens[6])
        qx = float(tokens[7])
        qy = float(tokens[8])
        qz = float(tokens[9])

        yaw = quat_to_yaw(qw, qx, qy, qz)

        keyframes[kf_id] = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'T': pose_to_T(x, y, yaw)
        }

        xs.append(x)
        ys.append(y)




    # Read loop closures
    evaluations = []

    with open(args.loop_closures_filename, 'r') as f:
        lines = f.readlines()

    for line in lines[1:]:  # skip header
        tokens = line.strip().split(',')
        if len(tokens) < 10:
            continue

        j = int(tokens[0])
        i = int(tokens[1])

        dx = float(tokens[2])
        dy = float(tokens[3])
        qw = float(tokens[5])
        qx = float(tokens[6])
        qy = float(tokens[7])
        qz = float(tokens[8])
        score = float(tokens[9])

        yaw_lc = quat_to_yaw(qw, qx, qy, qz)
        T_lc = pose_to_T(dx, dy, yaw_lc)

        Ti = keyframes[i]['T']
        Tj = keyframes[j]['T']

        # Expected relative transform
        T_expected = np.linalg.inv(Ti) @ Tj

        # Error transform
        T_err = np.linalg.inv(T_lc) @ T_expected

        trans_error = np.linalg.norm(T_err[:2, 2])
        rot_error = abs(np.arctan2(T_err[1, 0], T_err[0, 0]))

        coherent = (trans_error < 0.5) and (rot_error < np.deg2rad(10))

        evaluations.append((
            i, j,
            dx, dy, yaw_lc,
            T_expected[0, 2], T_expected[1, 2],
            trans_error,
            np.rad2deg(rot_error),
            score,
            coherent
        ))

    # Write evaluation file
    with open("./output/localization/loop_closures_evaluation.csv", "w") as f:
        f.write(
            "keyframe_i,keyframe_j,"
            "lc_dx,lc_dy,lc_dyaw,"
            "expected_dx,expected_dy,"
            "translation_error,rotation_error_deg,"
            "score,coherent\n"
        )

        for e in evaluations:
            f.write(
                f"{e[0]},{e[1]},"
                f"{e[2]:.3f},{e[3]:.3f},{e[4]:.3f},"
                f"{e[5]:.3f},{e[6]:.3f},"
                f"{e[7]:.3f},{e[8]:.3f},"
                f"{e[9]:.3f},{int(e[10])}\n"
            )

    # Plot
    plt.figure(figsize=(10, 8))
    plt.plot(xs, ys, '-k', label="Keyframes")

    for e in evaluations:
        i, j = e[0], e[1]

        xi, yi = keyframes[i]['x'], keyframes[i]['y']
        xj, yj = keyframes[j]['x'], keyframes[j]['y']
        yawi = keyframes[i]['yaw']

        dx, dy = e[2], e[3]

        # Loop closure link (i -> j)
        if e[10]:
            plt.plot([xi, xj], [yi, yj], 'g--', alpha=0.6)
            arrow_color = 'g'
        else:
            plt.plot([xi, xj], [yi, yj], 'r--', alpha=0.6)
            arrow_color = 'r'

        # Transform arrow from keyframe i
        dx_w = np.cos(yawi) * dx - np.sin(yawi) * dy
        dy_w = np.sin(yawi) * dx + np.cos(yawi) * dy

        plt.arrow(
            xi, yi,
            dx_w, dy_w,
            head_width=0.05,
            length_includes_head=True,
            color=arrow_color,
            alpha=0.8
        )


    plt.axis('equal')
    plt.grid()
    plt.legend()
    plt.title("Loop Closures Evaluation (green = coherent)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()


if __name__ == "__main__":
    main()
