# postgres.py
# PostgreSQL DB와 상호작용하는 모듈
# 기능
# - 데이터베이스 연결
# - 재고 조회

import psycopg2


class PostgreSQL:
    def __init__(self, connection_info):
        self._connection_info = connection_info

    def connect(self):
        return psycopg2.connect(
            host=self._connection_info.host,
            port=self._connection_info.port,
            dbname=self._connection_info.database,
            user=self._connection_info.user,
            password=self._connection_info.password,
        )


class InventoryRepository:
    def __init__(self, database):
        self._database = database

    def find_all(self):
        query = """
            SELECT
                i.item_id,
                i.item_code,
                i.item_name,
                v.quantity,
                v.updated_at
            FROM item AS i
            JOIN inventory AS v USING (item_id)
            ORDER BY i.item_id
        """

        with self._database.connect() as connection:
            with connection.cursor() as cursor:
                cursor.execute(query)
                return cursor.fetchall()

    def decrement(self, item_code):
        query = """
            UPDATE inventory AS inventory
            SET quantity = inventory.quantity - 1,
                updated_at = NOW()
            FROM item
            WHERE item.item_id = inventory.item_id
            AND item.item_code = %s
            AND inventory.quantity > 0
            RETURNING
                inventory.item_id,
                inventory.quantity,
                inventory.updated_at
        """

        with self._database.connect() as connection:
            with connection.cursor() as cursor:
                cursor.execute(query, (item_code,))
                result = cursor.fetchone()

        return result
