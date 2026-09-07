// 환경변수 읽기
const databaseName = process.env.MONGO_INITDB_DATABASE;

if (!databaseName) {
  throw new Error("MONGO_INITDB_DATABASE must be set");
}

// 초기화 대상 DB 선택
const appDatabase = db.getSiblingDB(databaseName);

// 음성 명령: task_id로 한 번만 저장
// 고유 인덱스 생성
appDatabase.commands.createIndex(
  { task_id: 1 },
  {
    name: "uq_commands_task_id",
    unique: true,
  },
);

// 키트 실행: task_id로 한 번만 생성
appDatabase.kit_executions.createIndex(
  { task_id: 1 },
  {
    name: "uq_kit_executions_task_id",
    unique: true,
  },
);

// 컴포넌트 실행: task_id + component_index로 식별
appDatabase.component_executions.createIndex(
  {
    task_id: 1,
    component_index: 1,
  },
  {
    name: "uq_component_executions_task_component",
    unique: true,
  },
);