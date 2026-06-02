import argparse
import os
import cv2


def cm_to_px(cm, dpi):
    return int(round(cm / 2.54 * dpi))


def create_charuco_board(squares_x, squares_y, square_len_cm, marker_len_cm, aruco_dict):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            squares_x, squares_y,
            square_len_cm, marker_len_cm,
            aruco_dict
        )
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_len_cm, marker_len_cm,
            aruco_dict
        )
    raise RuntimeError("CharucoBoard not available. Install opencv-contrib-python.")


def draw_charuco_board(board, size):
    if hasattr(board, "draw"):
        return board.draw(size)
    if hasattr(board, "generateImage"):
        return board.generateImage(size)
    raise RuntimeError("Charuco board draw API not available.")


def main():
    parser = argparse.ArgumentParser(description="Generate a Charuco board for calibration")
    parser.add_argument("--squares-x", type=int, default=5)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-len-cm", type=float, default=4.0)
    parser.add_argument("--marker-len-cm", type=float, default=3.0)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--output", default="calibration/charuco_5x7_4cm.png")
    args = parser.parse_args()

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = create_charuco_board(
        args.squares_x, args.squares_y,
        args.square_len_cm, args.marker_len_cm,
        aruco_dict
    )

    width_cm = args.squares_x * args.square_len_cm
    height_cm = args.squares_y * args.square_len_cm
    w_px = cm_to_px(width_cm, args.dpi)
    h_px = cm_to_px(height_cm, args.dpi)

    img = draw_charuco_board(board, (w_px, h_px))
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    cv2.imwrite(args.output, img)

    print(f"Saved: {args.output}")
    print(f"Size: {width_cm:.1f}cm x {height_cm:.1f}cm at {args.dpi} DPI")


if __name__ == "__main__":
    main()
