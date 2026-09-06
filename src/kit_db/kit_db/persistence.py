# persistence.py
# message_mapper를 통해 ROS 메시지를 변환하는 기능을 제공하는 모듈

from kit_db.message_mapper import (
    command_document,
    component_document,
    task_status_update,
)

class PersistenceService:
    def __init__(self, mongo_repository, inventory_repository):
        self._mongo_repository = mongo_repository
        self._inventory_repository = inventory_repository

    # ROS 메시지를 MongoDB에 저장하는 메서드들
    # 1. record_command: CommandResult.msg -> commands document에 저장
    def record_command(self, message):
        document = command_document(message)
        return self._mongo_repository.save_command(document)

    # 2. record_task_status: TaskStatus.msg -> tasks document에 저장
    def record_task_status(self, message):
        update = task_status_update(message)

        return self._mongo_repository.update_task_status(
            message.task_id,
            update,
        )

    # 3. record_component: ComponentResult.msg -> components document에 저장
    def record_component(self, message):
        document = component_document(message)
        result = self._mongo_repository.save_component(document)

        if (
            result.upserted_id is not None
            and document['status'] == 'SUCCESS'
        ):
            self._inventory_repository.decrement(
                document['class_name']
            )
        return result

