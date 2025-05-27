# TaskFinished.msg
# Сообщает аллокатору, что робот действительно доехал до точки возврата

uint8                robot_id      # ID робота, выполнившего задачу
builtin_interfaces/Time stamp      # время завершения
uint8                task_id       # (опционально) индекс задачи в батче
