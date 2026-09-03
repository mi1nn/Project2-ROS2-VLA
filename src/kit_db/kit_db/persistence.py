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
        return self._upsert(
            'commands',
            {'task_id': message.task_id},
            {'$set': document},
        )

    def record_task_status(self, message):
        update = task_status_update(message)
        return self._upsert(
            'kit_executions',
            {'task_id': message.task_id},
            update,
        )

    def record_component(self, message):
        document = component_document(message)
        result = self._upsert(
            'component_executions',
            {
                'task_id': message.task_id,
                'component_index': message.component_index,
            },
            {'$set': document},
        )

        if should_decrement_inventory(
            document['status'], document['attempts']
        ):
            self._inventory_repository.decrement(document['class_name'])

        return result

    def _upsert(self, collection_name, identity, update):
        return self._mongo_database[collection_name].update_one(
            identity,
            update,
            upsert=True,
        )
