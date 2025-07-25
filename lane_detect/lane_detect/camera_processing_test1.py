import numpy as np
import cv2

class CameraProcessing:
    def __init__(self):
        self.GaussianBlur = 3 # 커질수록 더 부드러워짐
        self.LightRemove = 9 # 15 -> 9 조명 제거 강도(작아질 수록 많이 없어짐)
        self.MedianBlur = 5 # 3 -> 5
        self.bin_threshold = 80 # 10 -> 80
        self.kernels = {
            'vertical': np.array([[-1,  0,  1],
                                  [-2,  0,  2],
                                  [-1,  0,  1]]),
            'diagonal_1': np.array([[ 0,  1,  2],
                                    [-1,  0,  1],
                                    [-2, -1,  0]]),
            'diagonal_2': np.array([[ 2,  1,  0],
                                    [ 1,  0, -1],
                                    [ 0, -1, -2]])
        }

    def process_image(self, img):
        if img is None:
            return None

        img = self.remove_lighting(img)
        # _, img = cv2.threshold(img, self.bin_threshold, 255, cv2.THRESH_BINARY)
        _, img = cv2.threshold(img, self.bin_threshold, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)


        img = cv2.medianBlur(img, self.MedianBlur)

        # # 마우스 클릭 콜백 함수
        # def mouse_callback(event, x, y, flags, param):
        #     if event == cv2.EVENT_LBUTTONDOWN:
        #         print(f"Clicked position: x={x}, y={y}")

        # # 창 생성 및 콜백 등록
        # cv2.namedWindow('Image')
        # cv2.setMouseCallback('Image', mouse_callback)

        # while True:
        #     cv2.imshow('Image', img)
        #     if cv2.waitKey(1) & 0xFF == 27:  # ESC 키 누르면 종료
        #         break

        # cv2.destroyAllWindows()
        ########################
        kernel = np.ones((3,3), np.uint8)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=1)

        img = self.warp(img)
        filtered, img = self.choose_filtered_img(img)
        return img, filtered

    def choose_filtered_img(self, img):
        max_edge_strength = -1
        best_filtered_img = None
        best_kernel_name = None

        for kernel_name, kernel in self.kernels.items():
            filtered_img = cv2.filter2D(img, -1, kernel)
            edge_strength = np.sum(np.abs(filtered_img))

            if edge_strength > max_edge_strength:
                max_edge_strength = edge_strength
                best_filtered_img = filtered_img
                best_kernel_name = kernel_name

        return best_kernel_name, best_filtered_img

    def remove_lighting(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # added


        ## Yellow
        # lower_yellow = np.array([15,70,70])
        # upper_yellow = np.array([35,255,255])
        # yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # img[yellow_mask > 0] = [0, 0, 0] # added end

        # White
        lower_white = np.array([0,0,200])
        upper_white = np.array([180,30,255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        img[white_mask > 0] = [0,0,0]

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.LightRemove, self.LightRemove), 0)
        # diff_img = cv2.subtract(gray, blurred)
        # diff_img = cv2.addWeighted(gray, 1.5, blurred, -0.5, 0)
        diff_img = cv2.addWeighted(gray, 1.2, blurred, -0.2, 0)
        normalized_img = cv2.normalize(diff_img, None, 0, 255, cv2.NORM_MINMAX)

        # # new added
        # img_colored = cv2.cvtColor(normalized_img, cv2.COLOR_GRAY2BGR)
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # added
        # # lower_yellow = np.array([15,70,70])
        # # upper_yellow = np.array([35,255,255])
        # # yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # lower_white = np.array([0,0,180])
        # upper_white = np.array([180,50,255])
        # white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # # result = cv2.inpaint(img_colored, yellow_mask, 5, cv2.INPAINT_TELEA)
        # result = cv2.inpaint(img_colored, white_mask, 5, cv2.INPAINT_TELEA)

        # result_gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        # return result_gray
        return normalized_img
    def warp(self, img):
        h, w = img.shape[:2]
        # src = np.float32([
        #     [170, 300],      # top-left
        #     [40, 380],     # bottom-left
        #     [500, 300],     # top-right
        #     [630, 380],    # bottom-right
        # ])
        src = np.float32([
            [190, 85],      # top-left #
            [156, 370],     # bottom-left
            [508, 85],     # top-right
            [528, 370],    # bottom-right ###  optimized
        ])
        # src = np.float32([ # 480, 640
        #     [170, 85],      # top-left #
        #     [156, 370],     # bottom-left
        #     [430, 85],     # top-right
        #     [452, 370],    # bottom-right
        # ])
        dst = np.float32([
            [140, 0],      # top-left
            [140, 480],    # bottom-left
            [500, 0],      # top-right
            [500, 480],    # bottom-right   #### optimized
        ])
        # dst = np.float32([
        #     [150, 0],      # top-left
        #     [150, 480],    # bottom-left
        #     [490, 0],      # top-right
        #     [490, 480],    # bottom-right
        # ])
        M = cv2.getPerspectiveTransform(src, dst)
        warped_img = cv2.warpPerspective(img, M, (w, h))
        return warped_img
