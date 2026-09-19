/**
 * UI 전용 타입 정의
 * - 이 파일은 BLE 통신 로직을 포함하지 않습니다.
 * - 기존 BLE 코드에서 이미 관리 중인 상태값들을 "그대로" 이 타입에 맞춰
 *   props로 넘겨주기만 하면 됩니다.
 */

/** BLE / HC-12 공용 연결 상태 */
export type ConnectionStatus = 'connected' | 'connecting' | 'disconnected';

/**
 * 로봇 주행 상태
 * - 요청하신 대로 대기 / 주행 중 / 정지 / 도착 완료 / 복귀 중 / 복귀 완료 까지
 *   미리 정의해 두었습니다. 지금 당장 RETURN 기능이 없다면 returning/returnCompleted는
 *   사용하지 않아도 되고, 추후 그대로 사용하시면 됩니다.
 */
export type RobotStatus =
  | 'idle'            // 대기
  | 'driving'         // 주행 중
  | 'stopped'         // 정지
  | 'arrived'         // 도착 완료
  | 'returning'       // 복귀 중
  | 'returnCompleted'; // 복귀 완료

/**
 * 확장 예정 경고/오류 플래그.
 * 지금은 값이 없어도(undefined) 되고, 센서가 추가되는 대로 하나씩 true/false를
 * 채워 넣으면 자동으로 경고 배너에 표시됩니다.
 */
export interface WarningFlags {
  obstacleDetected?: boolean;   // 장애물 감지
  communicationLost?: boolean;  // 통신 끊김 (BLE/HC-12 등 상위 레벨 오류)
  laneDetectionFailed?: boolean; // 차선 인식 실패
  tiltDetected?: boolean;       // 전도(넘어짐) 감지
}

/**
 * RobotControlScreen 이 받는 전체 props.
 * - 값(상태)과 콜백(동작)을 분리해서, 기존 BLE 훅/상태와 그대로 연결할 수 있게 했습니다.
 * - onReturn, onResetDistance 는 아직 기능이 없다면 넘기지 않아도 됩니다(선택 prop).
 *   전달하지 않으면 해당 버튼은 자동으로 숨김/비활성 처리됩니다.
 */
export interface RobotControlScreenProps {
  // ---- 상태값 (기존 BLE 로직에서 그대로 전달) ----
  connectionStatus: ConnectionStatus;
  /** HC-12 무선 모듈 상태. 아직 없으면 생략 가능 */
  hc12Status?: ConnectionStatus;
  robotStatus: RobotStatus;

  targetDistance: number;   // 목표 거리 (m)
  currentDistance: number;  // 현재 이동 거리 (m) - 지금은 10초 타이머 기반, 추후 엔코더 값으로 교체 예정

  /** 확장 예정 경고 상태 (지금은 전달 안 해도 됨) */
  warnings?: WarningFlags;

  /** 디바이스 이름 등 부가 정보 표시용 (선택) */
  deviceName?: string;

  // ---- 동작 콜백 (기존 BLE 함수 그대로 연결) ----
  onConnect: () => void;
  onDisconnect: () => void;
  onTargetDistanceChange: (meters: number) => void;
  onStart: () => void;
  onStop: () => void;

  /** 복귀 기능 - 아직 미구현이면 prop 자체를 넘기지 마세요 (버튼 자동 숨김) */
  onReturn?: () => void;
  /** 거리 초기화 - 아직 미구현이면 prop 자체를 넘기지 마세요 (버튼 자동 숨김) */
  onResetDistance?: () => void;
}
