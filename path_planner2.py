import numpy as np
import cv2
from queue import Queue
import tkinter as tk
from tkinter import filedialog
from image_clicker import capture_image

class ImagePathfinder:
    def __init__(self, image_path, buffer_size: int = 10, obstacle_buffer: int = 20):
        self.image_path = image_path
        self.original_img = cv2.imread(self.image_path)
        self.gray_img = cv2.cvtColor(self.original_img, cv2.COLOR_BGR2GRAY)

        # Preprocessing
        blurred = cv2.GaussianBlur(self.gray_img, (5, 5), 0)
        _, binary_img = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        kernel = np.ones((5, 5), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)

        dilated_img = cv2.dilate(binary_img, np.ones((buffer_size, buffer_size), np.uint8), iterations=1)
        obstacle_expanded = cv2.dilate(dilated_img, np.ones((obstacle_buffer, obstacle_buffer), np.uint8), iterations=1)

        self.grid = (obstacle_expanded == 0).astype(int)
        self.rows, self.cols = self.grid.shape
        self.start, self.end = self.select_points()

#    def upload_image(self):
#        root = tk.Tk()
#        root.withdraw()
#        return filedialog.askopenfilename(title='Select an Image', filetypes=[('Image Files', '*.png;*.jpg;*.jpeg;*.bmp')])

    def select_points(self):
        start, end = None, None

        def on_mouse(event, x, y, flags, param):
            nonlocal start, end
            if event == cv2.EVENT_LBUTTONDOWN:
                if start is None:
                    start = (y, x)
                    cv2.circle(self.original_img, (x, y), 5, (0, 255, 0), -1)
                elif end is None:
                    end = (y, x)
                    cv2.circle(self.original_img, (x, y), 5, (0, 0, 255), -1)
                    cv2.imshow('Select Points', self.original_img)

        cv2.imshow('Select Points', self.original_img)
        cv2.setMouseCallback('Select Points', on_mouse)
        print("Click to select start (green) and end (red) points")
        while start is None or end is None:
            cv2.waitKey(1)
        cv2.destroyAllWindows()
        return start, end

    def get_neighbors(self, node):
        row, col = node
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Only horizontal and vertical moves
        neighbors = [(row+dr, col+dc) for dr, dc in directions
                     if 0 <= row+dr < self.rows and 0 <= col+dc < self.cols and self.grid[row+dr, col+dc] == 1]
        return neighbors

    def find_simple_path(self):
        start, goal = self.start, self.end
        if self.grid[start] == 0 or self.grid[goal] == 0:
            print("Start or goal position is in an obstacle!")
            return []

        queue = Queue()
        queue.put(start)
        came_from = {start: None}

        while not queue.empty():
            current = queue.get()
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                return self.extract_key_points(path[::-1])

            for neighbor in self.get_neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    queue.put(neighbor)

        print("No path found!")
        return []

    def extract_key_points(self, path):
        if not path:
            return []
        key_points = [path[0]]

        for i in range(1, len(path) - 1):
            prev, curr, next_ = path[i - 1], path[i], path[i + 1]
            if (prev[0] != next_[0]) and (prev[1] != next_[1]):  # Change in direction
                key_points.append(curr)

        key_points.append(path[-1])
        return key_points

    def animate_path(self, path):
        result_img = self.original_img.copy()
        for i in range(len(path) - 1):
            start_point, end_point = (path[i][1], path[i][0]), (path[i+1][1], path[i+1][0])
            cv2.line(result_img, start_point, end_point, (255, 128, 0), 2)
            cv2.imshow('Path Animation', result_img)
            cv2.waitKey(50)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
# Usage Example
# Capture the image and get the file path
image_path = capture_image()

# Only proceed if the image was captured successfully

if image_path:
    pathfinder = ImagePathfinder(image_path=image_path, buffer_size=10, obstacle_buffer=20)
    path = pathfinder.find_simple_path()
    if path:
        path_matrix = np.array(path)
        pathfinder.animate_path(path)
        print(path_matrix)
    else:
        print("No valid path found!")
else:
    print("Failed to capture image.")