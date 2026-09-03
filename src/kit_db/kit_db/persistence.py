"""Coordinate mapped ROS messages and database writes."""

from kit_db.inventory_policy import should_decrement_inventory
from kit_db.message_mapping import (
    command_document,
    component_document,
    task_status_update,
)


class PersistenceService:
    """Persist kit messages without depending on ROS callback machinery."""

    def __init__(self, mongo_database, inventory_repository):
        self._mongo_database = mongo_database
        self._inventory_repository = inventory_repository

    def record_command(self, message):
        document = command_document(message)
        return self._mongo_database['commands'].update_one(
            {'task_id': message.task_id},
            {'$set': document},
            upsert=True,
        )

    def record_task_status(self, message):
        update = task_status_update(message)
        return self._mongo_database['kit_executions'].update_one(
            {'task_id': message.task_id},
            update,
            upsert=True,
        )

    def record_component(self, message):
        document = component_document(message)
        result = self._mongo_database['component_executions'].update_one(
            {
                'task_id': message.task_id,
                'component_index': message.component_index,
            },
            {'$set': document},
            upsert=True,
        )

        if should_decrement_inventory(
            document['status'], document['attempts']
        ):
            self._inventory_repository.decrement(document['class_name'])

        return result
