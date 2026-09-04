# 환경 변수에서 데이터베이스 연결 설정을 로드

import os
from dataclasses import dataclass, field

# 필수 환경변수 검사
def _required(name):
    value = os.environ.get(name)
    if value is None or not value.strip():
        raise ValueError(f'{name} environment variable is required')
    return value

# 포트 검사
def _port(name, default):
    value = os.environ.get(name, str(default))

    try:
        port = int(value)
    except ValueError as error:
        raise ValueError(f'{name} must be an integer') from error

    if not 1 <= port <= 65535:
        raise ValueError(f'{name} must be between 1 and 65535')

    return port

# PostgreSQL 접속에 필요한 환경변수를 하나의 객체로 묶어 관리
# 설정 객체값 변경 방지
@dataclass(frozen=True)
class PostgreSQLConfig:
    """PostgreSQL connection settings."""

    host: str
    port: int
    database: str
    user: str
    password: str = field(repr=False)

    # 환경변수에서 PostgreSQL Config 설정을 로드하는 클래스 메서드
    @classmethod
    def from_environment(cls):
        """Create settings for a database exposed on the local host."""
        return cls(
            host=os.environ.get('POSTGRES_HOST', '127.0.0.1'),
            port=_port('POSTGRES_PORT', 5432),
            database=_required('POSTGRES_DB'),
            user=_required('POSTGRES_USER'),
            password=_required('POSTGRES_PASSWORD'),
        )

# MongoDB 접속에 필요한 환경변수를 하나의 객체로 묶어 관리
@dataclass(frozen=True)
class MongoDBConfig:
    """MongoDB connection settings."""

    host: str
    port: int
    database: str
    user: str
    password: str = field(repr=False)   # 비밀번호 노출 방지
    authentication_database: str = 'admin'

    # 환경변수에서 MongoDB Config 설정을 로드하는 클래스 메서드
    @classmethod
    def from_environment(cls):
        """Create settings for a database exposed on the local host."""
        return cls(
            host=os.environ.get('MONGO_HOST', '127.0.0.1'),
            port=_port('MONGO_PORT', 27017),
            database=_required('MONGO_DATABASE'),
            user=_required('MONGO_ROOT_USER'),
            password=_required('MONGO_ROOT_PASSWORD'),
        )

# 전체 데이터베이스 설정을 하나의 객체로 묶어 관리
@dataclass(frozen=True)
class DatabaseConfig:
    """All database settings used by the DB node."""

    postgres: PostgreSQLConfig
    mongodb: MongoDBConfig

    # 두 DB 설정을 하나로 묶어 관리하는 메서드
    @classmethod
    def from_environment(cls):
        """Load and validate all database settings."""
        return cls(
            postgres=PostgreSQLConfig.from_environment(),
            mongodb=MongoDBConfig.from_environment(),
        )
