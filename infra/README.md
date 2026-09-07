# Local database infrastructure

PostgreSQL은 품목과 현재 재고를, MongoDB는 명령과 키트·컴포넌트 실행 결과를
저장한다. 두 DB는 Docker Compose로 로컬에서 실행하며 호스트의 loopback
주소에만 포트를 연다.

## 구성

| DB | 저장 대상 | 초기화 파일 |
| --- | --- | --- |
| PostgreSQL 16 | `item`, `inventory` | `postgres/init/001_schema.sql`, `postgres/init/002_seed.sql` |
| MongoDB 7 | `commands`, `kit_executions`, `component_executions` | `mongodb/init/001_initialize.js` |

MongoDB 초기화 파일은 문서 데이터를 생성하지 않고, 컬렉션별 고유
인덱스만 생성한다.

## 사전 요구 사항

- Docker Engine
- Docker Compose

버전을 확인한다.

```bash
docker --version
docker compose version
```

## 최초 구축

저장소 루트에서 예시 환경 파일을 복사한다.

```bash
cp .env.example .env
```

`.env`의 `change_me` 비밀번호를 로컬 개발용 값으로 변경한다. `.env`는 Git에
커밋하지 않는다.

```dotenv
POSTGRES_USER=kit_app
POSTGRES_PASSWORD=change_me
POSTGRES_DB=kit_system
POSTGRES_PORT=5432

MONGO_ROOT_USER=kit_admin
MONGO_ROOT_PASSWORD=change_me
MONGO_DATABASE=kit_system
MONGO_PORT=27017
```

실행 전에 Compose 설정과 필수 환경변수를 검사한다.

```bash
docker compose config --quiet
```

DB를 백그라운드에서 실행한다.

```bash
docker compose up -d
```

## 실행 상태 검사

```bash
docker compose ps
```

`postgres`와 `mongodb` 모두 `healthy`여야 한다. 시작에 실패했거나
`unhealthy`인 경우 로그를 확인한다.

```bash
docker compose logs postgres
docker compose logs mongodb
```

`address already in use` 오류가 발생한 경우 `.env`의 호스트 포트만 변경한다.
(호스트에서 기본 포트를 이미 사용 중)

```dotenv
POSTGRES_PORT=5433
MONGO_PORT=27018
```

컨테이너 내부 포트 `5432`, `27017`은 변경되지 않는다.

## PostgreSQL 검사

### 방법 1 : 접속

```bash
docker compose exec postgres \
  sh -c 'psql -U "$POSTGRES_USER" -d "$POSTGRES_DB"'
```

`psql` 내부에서 테이블과 seed 데이터를 확인한다.

```sql
\dt

SELECT i.item_id, i.item_code, i.item_name,
       v.quantity, v.updated_at
FROM item AS i
JOIN inventory AS v USING (item_id)
ORDER BY i.item_id;
```

접속을 종료한다.

```sql
\q
```

### 방법 2 : 비대화형 검사

```bash
docker compose exec postgres \
  sh -c 'psql -U "$POSTGRES_USER" -d "$POSTGRES_DB" -P pager=off -c \
  "SELECT i.item_id, i.item_code, v.quantity
   FROM item AS i
   JOIN inventory AS v USING (item_id)
   ORDER BY i.item_id;"'
```

9개 품목과 seed 수량이 출력되면 PostgreSQL 초기화가 정상이다.

## MongoDB 검사

### 방법 1 : 접속

```bash
docker compose exec mongodb \
  sh -c 'mongosh \
    --username "$MONGO_INITDB_ROOT_USERNAME" \
    --password "$MONGO_INITDB_ROOT_PASSWORD" \
    --authenticationDatabase admin \
    "$MONGO_INITDB_DATABASE"'
```

`mongosh` 내부에서 DB, 컬렉션, 인덱스와 문서 수를 확인한다.

```javascript
db.getName()
show collections

db.commands.getIndexes()
db.kit_executions.getIndexes()
db.component_executions.getIndexes()

db.commands.countDocuments()
db.kit_executions.countDocuments()
db.component_executions.countDocuments()
```

작업 데이터를 저장하지 않은 초기 상태에서 각 문서 수는 `0`이다. 접속을
종료한다.

```javascript
exit
```

### 방법 2 : 비대화형 검사

```bash
docker compose exec mongodb \
  sh -c 'mongosh \
    --quiet \
    --username "$MONGO_INITDB_ROOT_USERNAME" \
    --password "$MONGO_INITDB_ROOT_PASSWORD" \
    --authenticationDatabase admin \
    "$MONGO_INITDB_DATABASE" \
    --eval "
      printjson({
        database: db.getName(),
        collections: db.getCollectionNames(),
        counts: {
          commands: db.commands.countDocuments(),
          kit_executions: db.kit_executions.countDocuments(),
          component_executions:
            db.component_executions.countDocuments()
        }
      });
    "'
```

## 중지와 재시작

데이터를 유지하면서 컨테이너를 중지한다.

```bash
docker compose stop
```

기존 데이터로 다시 시작한다.

```bash
docker compose up -d
```

컨테이너를 제거해도 named volume을 지우지 않으면 DB 데이터는 유지된다.

```bash
docker compose down
```

## 초기화 : 사용에 주의 필요!!

`postgres/init` SQL과 `mongodb/init` JavaScript는 각 DB의 데이터 디렉터리가
빈 상태일 때만 자동 실행된다. 초기화 파일을 수정했다고 기존 볼륨에
자동 반영되지 않는다.

### PostgreSQL만 초기화

> 아래 명령은 PostgreSQL의 모든 데이터를 영구적으로 삭제한다.

```bash
docker compose stop postgres
docker compose rm -f postgres
docker volume rm vla-kit-system_postgres_data
docker compose up -d postgres
```

### MongoDB만 초기화

> 아래 명령은 MongoDB의 모든 데이터를 영구적으로 삭제한다.

```bash
docker compose stop mongodb
docker compose rm -f mongodb
docker volume rm vla-kit-system_mongodb_data
docker compose up -d mongodb
```

### 두 DB 모두 초기화

> `down -v`는 PostgreSQL과 MongoDB의 모든 데이터를 영구적으로 삭제한다.

```bash
docker compose down -v
docker compose up -d
```

초기화 후 `docker compose ps`와 각 DB 검사 절차를 다시 실행한다.
