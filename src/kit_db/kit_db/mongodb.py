# mongodb.py
# DB 노드에서 사용되는 MongoDB 접속을 관리

# Python 애플리케이션과 MongoDB 서버 사이 연결을 관리
from pymongo import MongoClient


class MongoDB:
    # MongoClient를 DB 노드가 사용하기 편한 평태로 감싸는 wrapper 클래스

    # 생성자 정보
    def __init__(self, connection_info):
        # MongoClient 생성
        self._client = MongoClient(
            host=connection_info.host,
            port=connection_info.port,
            username=connection_info.user,
            password=connection_info.password,
            authSource=connection_info.authentication_database,
            serverSelectionTimeoutMS=3000,  # 작업할 서버 선택 시간
            connectTimeoutMS=3000,          # 연결 시도 시간
        )
        # 애플리케이션 db 선택 - db 접근을 위한 객체 생성
        self._database = self._client[connection_info.database]

    # MongoDB 서버가 연결 가능한지 확인
    def ping(self):
        self._client.admin.command('ping')

    # 컬렉션 반환 - PyMongo의 컬렉션 객체
    def collection(self, name):
        return self._database[name]

    # MongoDB 연결 종료
    # DB 노드 종료 시 호출
    def close(self):
        self._client.close()


class MongoRepository:
    # 매핑된 메시지를 계약에 지정된 MongoDB 컬렉션에 저장
    def __init__(self, mongodb):
        self._commands = mongodb.collection('commands')
        self._kit_executions = mongodb.collection('kit_executions')
        self._component_executions = mongodb.collection(
            'component_executions'
        )

    # Command는 task_id 기준 최초 문서만 저장
    def save_command(self, document):
        return self._commands.update_one(
            {'task_id': document['task_id']},
            {'$setOnInsert': document},
            upsert=True,
        )

    # Kit 실행 문서는 task_id 기준으로 현재 상태와 이력을 갱신
    def update_task_status(self, task_id, update):
        return self._kit_executions.update_one(
            {'task_id': task_id},
            update,
            upsert=True,
        )

    # Component는 task_id와 component_index 기준 최초 문서만 저장
    def save_component(self, document):
        return self._component_executions.update_one(
            {
                'task_id': document['task_id'],
                'component_index': document['component_index'],
            },
            {'$setOnInsert': document},
            upsert=True,
        )
