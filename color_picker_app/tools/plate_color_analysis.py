def get_colors_from_file(img_path, offset=None):
    from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Tuple
    from pathlib import Path

    import cv2
    import numpy as np

    if TYPE_CHECKING:
        import numpy.typing as npt

    def _to_homogeneous(pts: "npt.NDArray[np.int64]") -> "npt.NDArray[np.float64]":
        *front, d = pts.shape
        points = np.ones((*front, d + 1))
        points[..., :-1] = pts
        return points

    def _homogenize(pts: "npt.NDArray[np.float64]") -> "npt.NDArray[np.float64]":
        *front, d = pts.shape
        pts = pts / pts[..., -1].reshape(*front, 1)
        return pts

    def _from_homogeneous(pts: "npt.NDArray[np.float64]") -> "npt.NDArray[np.float64]":
        return _homogenize(pts)[..., :-1]

    def _find_fiducials(
        img: cv2.Mat,
    ) -> Tuple[
        Tuple[
            "npt.NDArray[np.float32]",
            "npt.NDArray[np.float32]",
            "npt.NDArray[np.float32]",
            "npt.NDArray[np.float32]",
        ],
        "npt.NDArray[np.int32]",
    ]:
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img.copy()

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(
            gray  # , aruco_dict, parameters=parameters
        )
        return corners, ids

    def _plate_size(p: "npt.NDArray[np.int64]") -> np.float64:
        # Get a rough idea of how large a plate is (corner2corner)
        return np.linalg.norm([[p[0] - p[2]], [p[1] - p[3]]])

    def _orient(img: cv2.Mat) -> "npt.NDArray[np.int64]":
        """Finds the corners in image coordinate space of the largest fiducial."""
        # Find all of the fiducials
        corners, ids = _find_fiducials(img)
        assert (
            corners
        ), "Fiducial not found. Check if the ArUco tag is in the image and not mirrored."

        corners = np.concatenate(corners, axis=0)

        # Keep the largest fiducial and ignore all others
        largest = np.argmax(np.linalg.norm(corners[:, 0] - corners[:, 2], axis=1))
        c = corners[largest].astype(int)

        return c

    def _refine_angle(
        img: cv2.Mat, orientation: "npt.NDArray[np.int64]"
    ) -> Tuple[cv2.Mat, "npt.NDArray[np.int64]"]:
        # Find the angle of the fiducial
        f0x = orientation[3, 0]
        f0y = orientation[3, 1]
        fxx = orientation[2, 0]
        fxy = orientation[2, 1]
        theta_orig = np.arctan2(fxy - f0y, fxx - f0x)

        # Find the amount (<45º) to rotate the image that lines up the fiducial
        theta = theta_orig % (np.pi / 2)
        theta = theta if theta < np.pi / 4 else theta - (np.pi / 2)

        # Do the rotation
        (h, w) = img.shape[:2]
        M = cv2.getRotationMatrix2D((w / 2, h / 2), np.rad2deg(theta), 1.0)
        img = cv2.warpAffine(img, M, (w, h))

        # If the image is off by a 90º multiple rotation, fix that too
        if np.pi / 4 < theta_orig < np.pi * 3 / 4:
            img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if -np.pi / 4 > theta_orig > -np.pi * 3 / 4:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        if theta_orig > np.pi * 3 / 4 or theta_orig < -np.pi * 3 / 4:
            img = cv2.rotate(img, cv2.ROTATE_180)

        # Return the updated orientation and rotated image
        orientation = _orient(img)
        return img, orientation

    def _estimate_plates(
        img: cv2.Mat, orientation: "npt.NDArray[np.int64]", offset: dict
    ) -> "npt.NDArray[np.int64]":
        # Coordinates from the fiducial
        f0x = orientation[3, 0]
        f0y = orientation[3, 1]
        fxx = orientation[2, 0]
        fxy = orientation[2, 1]
        fyx = orientation[0, 0]
        fyy = orientation[0, 1]

        # Transformation matrix from fiducial to image coordinates
        M = np.array(
            [
                [fxx - f0x, fxy - f0y, 0],
                [fyx - f0x, fyy - f0y, 0],
                [f0x, f0y, 1],
            ]
        )

        # Fiducial coordinates of plate positions
        # (Based on the size and position of the fiducial)
        x0 = offset["x0"]
        y0 = offset["y0"]
        xx = offset["xx"]
        xy = offset["xy"]
        yy = offset["yy"]
        yx = offset["yx"]
        xp = offset["xp"]
        yp = offset["yp"]

        # Construct the bottom-left and top-right points of the plates
        pts = []
        for j in range(1):  # 4):
            for i in range(1):  # 3):
                pts.append(
                    [
                        x0 - xp + xx * i + yx * j,
                        y0 - yp + yy * j + xy * i,
                    ]
                )
                pts.append(
                    [
                        x0 + xp + xx * (i + 1) + yx * (j + 1),
                        y0 + yp + yy * (j + 1) + xy * (i + 1),
                    ]
                )
        pts = np.array(pts)
        # Convert to image coordinates
        pts = _from_homogeneous(_to_homogeneous(pts) @ M).astype(int)
        # Each plate is turned into a (x1, y1, x2, y2) tuple
        pts = pts.reshape((-1, 4))

        return pts

    def _errf(inp: List[np.float64], pts: "npt.NDArray[np.int64]") -> np.float64:
        x0, y0, xs, ys = inp

        # Transformation matrix from pixel space to 'plate' space
        M = np.array(
            [
                [xs, 0, x0],
                [0, ys, y0],
                [0, 0, 1],
            ]
        )

        # Convert points from pixel space to plate space
        p = np.linalg.inv(M) @ _to_homogeneous(pts[:, :-1]).T
        p = _from_homogeneous(p.T)

        # Find the distance to the nearest integer position
        x = p[:, 0]
        x = np.round(x) - x

        y = p[:, 1]
        y = np.round(y) - y

        # Find how close the points are to the nearest gridpoint
        d = np.linalg.norm([x, y], axis=0)

        # Return the average error
        return np.average(d)

    def _optimize(
        f: Callable[[List[np.float64], "npt.NDArray[np.int64]"], np.float64],
        est: np.float64,
        pts: "npt.NDArray[np.int64]",
    ) -> Tuple[np.float64, np.float64, np.float64, np.float64]:
        x0_best = None
        y0_best = None
        xs_best = None
        ys_best = None
        vl_best = np.inf

        # We have a good estimate, but refine it by brute force
        for xs in np.linspace(est * 0.96, est * 1.04, 9):
            for ys in np.linspace(xs * 0.98, xs * 1.02, 9):
                for x0 in np.linspace(0, xs, 9):
                    for y0 in np.linspace(0, ys, 9):
                        vl = f([x0, y0, xs, ys], pts)
                        if vl < vl_best:
                            x0_best = x0
                            y0_best = y0
                            xs_best = xs
                            ys_best = ys
                            vl_best = vl

        x0_best %= xs_best
        y0_best %= ys_best

        return x0_best, y0_best, xs_best, ys_best

    def _refine_plate(
        img: cv2.Mat, plate: "npt.NDArray[np.int64]"
    ) -> "npt.NDArray[np.float64]":
        # Find all of the circles in the image near the estimated well size
        radius = _plate_size(plate) / 55
        blur = cv2.GaussianBlur(img, (5, 5), 0.75)

        if len(blur.shape) == 3:
            gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        else:
            gray = blur.copy()

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=radius * 2.5,
            # param1=50,
            param1=80,
            param2=7,
            # param2=10,
            # minRadius=round(radius * 0.9),
            minRadius=round(radius * 0.95),
            # maxRadius=round(radius * 1.1),
            maxRadius=round(radius * 1.05),
        )

        if circles is None:
            return None

        # Filter out circles that are not near the current plate
        mnx = min(plate[0], plate[2])
        mxx = max(plate[0], plate[2])
        mny = min(plate[1], plate[3])
        mxy = max(plate[1], plate[3])
        pts = []
        for circle in np.squeeze(np.around(circles).astype(int), axis=0):
            if mxx > circle[0] > mnx and mxy > circle[1] > mny:
                pts.append(circle)
        pts = np.array(pts)

        # We want at least 3 circles to make a grid out of
        if pts.size <= 3:
            return None

        # Create an initial grid size estimate, and optimize it
        estimate = _plate_size(plate) / 16.37
        x0, y0, xs, ys = _optimize(_errf, estimate, pts)

        xmin_opt = {}
        for i in range(int((mnx - x0) / xs) - 10, int((mnx - x0) / xs) + 10):
            v1 = x0 + i * xs
            v2 = x0 + (i + 11) * xs
            a1 = v1 - mnx
            a2 = mxx - v2
            xmin_opt[i] = max(abs(a1), abs(a2))
        xmin = min(xmin_opt, key=xmin_opt.get)

        ymin_opt = {}
        for i in range(int((mny - y0) / ys) - 10, int((mny - y0) / ys) + 10):
            v1 = y0 + i * ys
            v2 = y0 + (i + 7) * ys
            a1 = v1 - mny
            a2 = mxy - v2
            ymin_opt[i] = max(abs(a1), abs(a2))
        ymin = min(ymin_opt, key=ymin_opt.get)

        # Translate the transformation matrix origin to start at the first well
        M = np.array(
            [
                [xs, 0, x0 + xmin * xs],
                [0, ys, y0 + ymin * ys],
                [0, 0, 1],
            ]
        )

        return M

    def _find_wells(
        img: cv2.Mat, plateM: "npt.NDArray[np.float64]"
    ) -> Dict[str, "npt.NDArray[np.int64]"]:
        if plateM is None:
            return []

        # Create points for each well position in plate coordinates
        pts = []
        for j in range(8):
            for i in range(12):
                pts.append([i, j])
        pts = np.array(pts)

        # Transform from plate coordinates to pixel coordinates
        pts = plateM @ _to_homogeneous(pts).T
        pts = _from_homogeneous(pts.T).astype(int)

        # Match well names with their pixel locations
        well_names = [a + b for a in "ABCDEFGH" for b in map(str, range(1, 13))]
        wells = {a: b for a, b in zip(well_names, pts)}
        return wells

    def _get_well_color(
        img: cv2.Mat, well: "npt.NDArray[np.int64]"
    ) -> "npt.NDArray[np.int64]":
        # Get the color at a position
        color = [a.item() for a in img[well[1], well[0]]]
        return np.array(color)

    def match_size(img: cv2.Mat, shape: Tuple[int, int]) -> cv2.Mat:
        min_img = min(img.shape[:2])
        max_img = max(img.shape[:2])

        min_out = min(shape)
        max_out = max(shape)

        ar_img = max_img / min_img
        ar_out = max_out / min_out

        if ar_img >= ar_out:
            resize_amount = min_out / min_img
            B = min_out
            C = max_out
        else:
            resize_amount = max_out / max_img
            B = max_out
            C = min_out

        img = cv2.resize(img, None, fx=resize_amount, fy=resize_amount)
        if img.shape[0] == B:
            diff = img.shape[1] - C
            a = diff // 2
            b = diff - a
            img = img[:, a:-b]
        else:
            diff = img.shape[0] - C
            a = diff // 2
            b = diff - a
            img = img[a:-b]

        if shape != img.shape[:2]:
            img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

        return img

    def get_colors(
        img: cv2.Mat, offset: dict = None
    ) -> Tuple[
        Dict[int, Dict[str, "npt.NDArray[np.int64]"]], cv2.Mat, cv2.Mat, cv2.Mat
    ]:
        """Analyzes an image for plates and determines the colors of all wells.
        >>> img = cv2.imread("sample.png") # Load image
        >>> img = match_size(img, (1280, 1920)) # Crop image
        >>> colors = get_colors(img) # Analyze image for colors
        >>> print(colors[1]["A1"])
        [161 163 215]
        """

        # Find the orientation of the image
        orientation = _orient(img)
        # Rotate the plates to be mostly lined up with the image axes
        img, orientation = _refine_angle(img, orientation)

        # Fiducial coordinates of plate positions
        # (Based on the size and position of the fiducial)
        if offset is None:
            offset = {
                "x0": 1.8,
                "y0": -0.15,
                "xx": 1.85,
                "xy": 0.04,
                "yy": 1.27,
                "yx": -0.01,
                "xp": 0.2,
                "yp": 0.1,
            }
        # Get an initial estimate of plate positions
        plates = _estimate_plates(img, orientation, offset)

        # Generate debug output with plate bounding box
        plate_img = img.copy()
        for i in range(0, len(plates)):
            x1, y1, x2, y2 = plates[i]
            cv2.line(plate_img, (x1, y1), (x1, y2), (255, 0, 255), 2)
            cv2.line(plate_img, (x1, y1), (x2, y1), (255, 0, 255), 2)
            cv2.line(plate_img, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.line(plate_img, (x2, y1), (x2, y2), (255, 0, 255), 2)
            cv2.line(plate_img, (x1, y2), (x2, y2), (255, 0, 255), 2)
            cv2.line(plate_img, (x1, y2), (x2, y1), (255, 0, 255), 2)

        platesD = {}
        for plate_idx, plate in enumerate(plates):
            # If any part of the plate lies outside of the image, ignore it
            lower_bound = plate > np.array([0, 0, 0, 0])
            upper_bound = plate < np.array(
                [img.shape[1], img.shape[0], img.shape[1], img.shape[0]]
            )
            if not np.all(lower_bound) or not np.all(upper_bound):
                continue

            # Use circle detection to find the orientation of the wells in the plate
            plateM = _refine_plate(img, plate)
            if plateM is None:
                print("WARNING: NO PLATE DETECTED, FALLING BACK TO DEFAULT LOCATION")
                plateM = np.array(
                    [
                        [45.54750043, 0, 637.66500602],
                        [0, 46.00297543, 414.02677891],
                        [0, 0, 1],
                    ]
                )

            # Find all of the well's pixel positions, and get the color there
            wells = _find_wells(img, plateM)
            wells_img = img.copy()
            for pt in wells.values():
                x, y = pt
                color = [a.item() for a in img[y, x]]
                cv2.circle(wells_img, (x, y), int(plateM[0, 0] / 2), color, -1)
            x1, y1, x2, y2 = plates[0]
            x1, x2, y1, y2 = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
            plate_only = img[y1:y2, x1:x2]
            plate_colors = wells_img[y1:y2, x1:x2]

            for wellname, well in wells.items():
                color = _get_well_color(img, well)
                wells[wellname] = color

            platesD[plate_idx + 1] = wells

        assert platesD, "Plate not detected at the expected location"

        return platesD, plate_img, plate_only, plate_colors

    img = cv2.imread(str(img_path))  # Load image
    assert img is not None, f'Image "{img_path}" not loaded. Check file name.'

    img = match_size(img, (1280, 1920))  # Crop image
    platesD, plate_img, plate_only, plate_colors = get_colors(
        img
    )  # Analyze image for colors

    base_path = img_path.parent
    cv2.imwrite(str(base_path / "plate_img.jpg"), plate_img)
    cv2.imwrite(str(base_path / "plate_only.jpg"), plate_only)
    cv2.imwrite(str(base_path / "plate_colors.jpg"), plate_colors)
    ##save side images

    return platesD
