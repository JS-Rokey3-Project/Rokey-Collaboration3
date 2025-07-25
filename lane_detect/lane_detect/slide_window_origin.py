import numpy as np
import cv2

class SlideWindow:
    def __init__(self):
        self.center_old = 320
        self.lane_width = 360
        self.left_old = self.center_old - self.lane_width / 180
        self.right_old = self.center_old + self.lane_width / 180

    def slide(self, img):
        height, width = img.shape
        c_img = np.dstack((img, img, img))

        window_height = 15
        window_width = 30
        minpix = 40
        n_windows = width // 2
        pts_center = np.array([[width // 2, 0], [width // 2, height]], np.int32)
        cv2.polylines(c_img, [pts_center], False, (0, 120, 120), 1)

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_center = self.center_old
        y_center = height // 2

        left_idx = right_idx = 0
        find_left = find_right = False

        left_start_x = right_start_x = 0
        dist_threshold = self.lane_width
        dist = None

        win_L_y_low = win_L_y_high = win_L_x_low = win_L_x_high = 0
        win_R_y_low = win_R_y_high = win_R_x_low = win_R_x_high = 0

        for _ in range(n_windows):
            if not find_left:
                win_L_y_low = y_center - window_height // 2
                win_L_y_high = y_center + window_height // 2
                win_L_x_high = x_center - left_idx * window_width
                win_L_x_low = x_center - (left_idx + 1) * window_width

            if not find_right:
                win_R_y_low = y_center - window_height // 2
                win_R_y_high = y_center + window_height // 2
                win_R_x_low = x_center + right_idx * window_width
                win_R_x_high = x_center + (right_idx + 1) * window_width

            cv2.rectangle(c_img, (win_L_x_low, win_L_y_low), (win_L_x_high, win_L_y_high), (0, 255, 0), 1)
            cv2.rectangle(c_img, (win_R_x_low, win_R_y_low), (win_R_x_high, win_R_y_high), (0, 0, 255), 1)

            good_left_inds = (
                (nonzeroy >= win_L_y_low) & (nonzeroy < win_L_y_high) &
                (nonzerox >= win_L_x_low) & (nonzerox < win_L_x_high)
            ).nonzero()[0]
            good_right_inds = (
                (nonzeroy >= win_R_y_low) & (nonzeroy < win_R_y_high) &
                (nonzerox >= win_R_x_low) & (nonzerox < win_R_x_high)
            ).nonzero()[0]

            if len(good_left_inds) > minpix and not find_left:
                find_left = True
                left_start_x = np.int32(np.mean(nonzerox[good_left_inds]))
                for ind in good_left_inds:
                    cv2.circle(c_img, (nonzerox[ind], nonzeroy[ind]), 1, (0, 255, 0), -1)
            else:
                left_idx += 1

            if len(good_right_inds) > minpix and not find_right:
                find_right = True
                right_start_x = np.int32(np.mean(nonzerox[good_right_inds]))
                for ind in good_right_inds:
                    cv2.circle(c_img, (nonzerox[ind], nonzeroy[ind]), 1, (0, 0, 255), -1)
            else:
                right_idx += 1

            if find_left and find_right:
                dist = right_start_x - left_start_x
                self.center_old = np.int32((right_start_x + left_start_x) / 2)
                if dist_threshold < dist < dist_threshold + 80:
                    self.left_old = left_start_x
                    self.right_old = right_start_x

                self.center_old = np.int32((left_start_x + right_start_x) / 2)
                return 'both', left_start_x, right_start_x, c_img

            if find_left:
                find_right = False
                dist_from_center = self.center_old - left_start_x
                dist_from_old = abs(left_start_x - self.left_old)

                if dist_from_center > 50 and dist_from_old < 55:
                    self.center_old = left_start_x + (dist_threshold // 2)
                    change = left_start_x - self.left_old
                    right_start_x = self.right_old + change
                    self.left_old = left_start_x
                    self.right_old = right_start_x
                    return 'left', left_start_x, right_start_x, c_img

            elif find_right:
                find_left = False
                dist_from_center = right_start_x - self.center_old
                dist_from_old = abs(right_start_x - self.right_old)

                if dist_from_center > 50 and dist_from_old < 55:
                    self.center_old = right_start_x - (dist_threshold // 2)
                    change = right_start_x - self.right_old
                    left_start_x = self.left_old + change
                    self.left_old = left_start_x
                    self.right_old = right_start_x
                    return 'right', left_start_x, right_start_x, c_img

        return False, self.left_old, self.right_old, c_img

    def lane_visualization(self, img, left_x_start, right_x_start):
        height, width = img.shape
        output_img = np.dstack((img, img, img))

        window_height = 30
        window_width = 30
        minpix = 40

        left_x_current = left_x_start
        right_x_current = right_x_start

        left_lane_pts = []
        right_lane_pts = []

        for y in range(height - window_height, 0, -window_height):
            win_left_x_low = int(left_x_current - window_width // 2)
            win_left_x_high = int(left_x_current + window_width // 2)
            win_y_low = int(y - window_height)
            win_y_high = int(y)

            win_right_x_low = int(right_x_current - window_width // 2)
            win_right_x_high = int(right_x_current + window_width // 2)

            cv2.rectangle(output_img, (win_left_x_low, win_y_low), (win_left_x_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(output_img, (win_right_x_low, win_y_low), (win_right_x_high, win_y_high), (0, 0, 255), 2)

            nonzero = img[win_y_low:win_y_high, win_left_x_low:win_left_x_high].nonzero()
            nonzerox_left = nonzero[1] + win_left_x_low
            nonzeroy_left = nonzero[0] + win_y_low

            nonzero = img[win_y_low:win_y_high, win_right_x_low:win_right_x_high].nonzero()
            nonzerox_right = nonzero[1] + win_right_x_low
            nonzeroy_right = nonzero[0] + win_y_low

            left_lane_pts.extend(list(zip(nonzerox_left, nonzeroy_left)))
            right_lane_pts.extend(list(zip(nonzerox_right, nonzeroy_right)))

            if len(nonzerox_left) > minpix:
                left_x_current = np.int32(np.mean(nonzerox_left))
            if len(nonzerox_right) > minpix:
                right_x_current = np.int32(np.mean(nonzerox_right))

        for point in left_lane_pts:
            cv2.circle(output_img, point, 1, (0, 255, 0), -1)
        for point in right_lane_pts:
            cv2.circle(output_img, point, 1, (0, 0, 255), -1)

        return output_img
