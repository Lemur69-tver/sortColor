import sim
import cv2
import numpy as np
import time


def connect_to_coppelia():
    sim.simxFinish(-1)
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if client_id == -1:
        raise ConnectionError("Не удалось подключиться к CoppeliaSim!")
    print("Успешное подключение, ID =", client_id)
    return client_id


def get_object_color(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    red_pixels = cv2.countNonZero(mask_red)
    green_pixels = cv2.countNonZero(mask_green)

    if red_pixels > green_pixels and red_pixels > 100:
        return "red"
    elif green_pixels > red_pixels and green_pixels > 100:
        return "green"
    return None


def main():
    client_id = connect_to_coppelia()

    # Получаем handles с проверкой ошибок
    res, cam_handle = sim.simxGetObjectHandle(client_id, "Vision_sensor", sim.simx_opmode_blocking)
    if res != sim.simx_return_ok:
        raise Exception("Не удалось получить handle камеры")

    res, conveyor_handle = sim.simxGetObjectHandle(client_id, "Conveyor_belt_joint", sim.simx_opmode_blocking)
    if res != sim.simx_return_ok:
        raise Exception("Не удалось получить handle конвейера")

    res, dobot_handle = sim.simxGetObjectHandle(client_id, "Dobot", sim.simx_opmode_blocking)
    if res != sim.simx_return_ok:
        raise Exception("Не удалось получить handle манипулятора")

    res, suction_handle = sim.simxGetObjectHandle(client_id, "suctionCup", sim.simx_opmode_blocking)
    if res != sim.simx_return_ok:
        raise Exception("Не удалось получить handle присоски")

    # Получаем разрешение камеры
    res, resolution, _ = sim.simxGetVisionSensorImage(client_id, cam_handle, 0, sim.simx_opmode_streaming)
    if res != sim.simx_return_ok:
        raise Exception("Не удалось получить разрешение камеры")

    time.sleep(1)  # Инициализация

    while True:
        # Получаем изображение с проверкой
        res, resolution, image = sim.simxGetVisionSensorImage(client_id, cam_handle, 0, sim.simx_opmode_buffer)
        if res == sim.simx_return_ok:
            try:
                img = np.array(image, dtype=np.uint8)
                img = img.reshape((resolution[1], resolution[0], 3))
                img = cv2.flip(img, 0)

                color = get_object_color(img)

                if color:
                    print(f"Обнаружен {color} объект! Останавливаю конвейер.")
                    sim.simxSetIntegerSignal(client_id, "stop_conveyor", 1, sim.simx_opmode_oneshot)

                    # Захват объекта
                    sim.simxSetIntegerSignal(client_id, "suction_active", 1, sim.simx_opmode_oneshot)
                    time.sleep(1)

                    # Получаем позицию с проверкой
                    res, obj_pos = sim.simxGetObjectPosition(client_id, suction_handle, -1, sim.simx_opmode_blocking)
                    if res == sim.simx_return_ok:
                        sim.simxSetObjectPosition(client_id, dobot_handle, -1,
                                                  [obj_pos[0], obj_pos[1], obj_pos[2] + 0.1],
                                                  sim.simx_opmode_oneshot)
                        time.sleep(1)

                        # Перенос в миску
                        bowl = bowl_red if color == "red" else bowl_green
                        res, target_pos = sim.simxGetObjectPosition(client_id, bowl, -1, sim.simx_opmode_blocking)
                        if res == sim.simx_return_ok:
                            sim.simxSetObjectPosition(client_id, dobot_handle, -1,
                                                      [target_pos[0], target_pos[1], target_pos[2] + 0.05],
                                                      sim.simx_opmode_oneshot)
                            time.sleep(2)

                    # Отпускание
                    sim.simxSetIntegerSignal(client_id, "suction_active", 0, sim.simx_opmode_oneshot)
                    time.sleep(1)

                    # Возврат
                    sim.simxSetObjectPosition(client_id, dobot_handle, -1, home_pos, sim.simx_opmode_oneshot)
                    time.sleep(1)

                    # Запуск конвейера
                    sim.simxSetIntegerSignal(client_id, "stop_conveyor", 0, sim.simx_opmode_oneshot)

            except Exception as e:
                print(f"Ошибка обработки изображения: {str(e)}")

        time.sleep(0.1)


if __name__ == "__main__":
    main()