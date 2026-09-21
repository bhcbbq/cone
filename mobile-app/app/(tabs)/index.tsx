import React, { useRef, useState } from 'react';
import {
  PermissionsAndroid,
  Platform,
} from 'react-native';

import {
  BleManager,
  Device,
  Subscription,
} from 'react-native-ble-plx';

import { RobotControlScreen } from '../../src/screens/ControlScreen';
import {
  ConnectionStatus,
  RobotStatus,
} from '../../src/types/robotUi';

// =====================================
// true  = 에뮬레이터 테스트
// false = 실제 폰 + ESP32 BLE
// =====================================
const TEST_MODE = false;

const manager = new BleManager();

const BLE_DEVICE_NAME =
  'ESP32_CONE';

const SERVICE_UUID =
  '6E400001-B5A3-F393-E0A9-E50E24DCCA9E';

// App -> ESP32 (Write)
const RX_UUID =
  '6E400002-B5A3-F393-E0A9-E50E24DCCA9E';

// ESP32 -> App (Notify)
const TX_UUID =
  '6E400003-B5A3-F393-E0A9-E50E24DCCA9E';

export default function HomeScreen() {
  // =========================
  // BLE
  // =========================

  const [device, setDevice] =
    useState<Device | null>(null);

  const notifySubscriptionRef =
    useRef<Subscription | null>(null);

  const [
    connectionStatus,
    setConnectionStatus,
  ] = useState<ConnectionStatus>('disconnected');

  // =========================
  // 거리
  // =========================

  // 입력창에 적혀 있는 다음 목표 거리
  const [
    targetDistance,
    setTargetDistance,
  ] = useState(0);

  // 현재/방금 끝난 주행의 전체 목표 거리
  const [
    activeTargetDistance,
    setActiveTargetDistance,
  ] = useState(0);

  const activeTargetDistanceRef =
    useRef(0);

  const [
    currentDistance,
    setCurrentDistance,
  ] = useState(0);

  const currentDistanceRef =
    useRef(0);

  // STOP 후 재출발할 때 이전까지 이동한 거리를 보존하기 위한 offset
  // 외부 ESP32는 START마다 엔코더 시작점을 새로 잡기 때문에,
  // ESP32가 보내는 DISTANCE 값은 각 구간의 상대 거리이다.
  const segmentOffsetRef =
    useRef(0);

  const [
    robotStatus,
    setRobotStatus,
  ] = useState<RobotStatus>('idle');

  // =========================
  // 상태/거리 업데이트 helper
  // =========================

  const updateCurrentDistance = (
    value: number
  ) => {
    const safeValue =
      Number.isFinite(value) && value >= 0
        ? value
        : 0;

    currentDistanceRef.current =
      safeValue;

    setCurrentDistance(
      safeValue
    );
  };

  const updateActiveTargetDistance = (
    value: number
  ) => {
    activeTargetDistanceRef.current =
      value;

    setActiveTargetDistance(
      value
    );
  };

  // =========================
  // 목표거리 입력 변경
  // =========================

  const changeTargetDistance = (
    value: number
  ) => {
    setTargetDistance(value);

    // 새 주행 전에는 진행 카드의 목표거리도 바로 반영
    if (robotStatus === 'idle') {
      updateActiveTargetDistance(
        value
      );
    }

    // stopped / arrived 상태에서는
    // 기존 주행 기록을 그대로 유지한다.
  };

  // =========================
  // Android BLE 권한
  // =========================

  const requestPermissions = async () => {
    if (TEST_MODE) {
      return true;
    }

    if (Platform.OS !== 'android') {
      return true;
    }

    if (Platform.Version >= 31) {
      const result =
        await PermissionsAndroid.requestMultiple([
          PermissionsAndroid.PERMISSIONS
            .BLUETOOTH_SCAN,
          PermissionsAndroid.PERMISSIONS
            .BLUETOOTH_CONNECT,
        ]);

      return (
        result[
          'android.permission.BLUETOOTH_SCAN'
        ] ===
          PermissionsAndroid.RESULTS.GRANTED &&
        result[
          'android.permission.BLUETOOTH_CONNECT'
        ] ===
          PermissionsAndroid.RESULTS.GRANTED
      );
    }

    const result =
      await PermissionsAndroid.request(
        PermissionsAndroid.PERMISSIONS
          .ACCESS_FINE_LOCATION
      );

    return (
      result ===
      PermissionsAndroid.RESULTS.GRANTED
    );
  };

  // =========================
  // ESP32 -> APP Notify 처리
  // =========================

  const handleEsp32Message = (
    rawMessage: string
  ) => {
    const message =
      rawMessage.trim();

    if (!message) {
      return;
    }

    console.log(
      'ESP32 -> APP:',
      message
    );

    // ---------------------------------
    // 실제 엔코더 거리
    // ESP32 예: DISTANCE:0.37
    // ---------------------------------
    if (
      message.startsWith(
        'DISTANCE:'
      )
    ) {
      const segmentDistance =
        parseFloat(
          message.substring(
            'DISTANCE:'.length
          )
        );

      if (
        !Number.isNaN(
          segmentDistance
        )
      ) {
        const totalDistance =
          segmentOffsetRef.current +
          segmentDistance;

        updateCurrentDistance(
          totalDistance
        );
      }

      return;
    }

    // ---------------------------------
    // 로봇 상태
    // ---------------------------------
    if (message === 'DRIVING') {
      setRobotStatus('driving');
      return;
    }

    if (message === 'STOPPED') {
      setRobotStatus('stopped');
      return;
    }

    if (message === 'ARRIVED') {
      // 목표 도달 시 화면은 정확히 100%로 맞춤
      updateCurrentDistance(
        activeTargetDistanceRef.current
      );

      setRobotStatus('arrived');
      return;
    }

    if (message === 'EMERGENCY') {
      setRobotStatus('stopped');
      return;
    }

    // 외부 ESP32가 아직 엔코더 데이터를 못 받은 경우
    if (message === 'ERROR:NO_ODOM') {
      console.log(
        'ESP32에 엔코더 데이터가 아직 없습니다.'
      );

      setRobotStatus('idle');
      return;
    }

    if (message === 'ERROR:NO_TARGET') {
      console.log(
        '목표거리가 설정되지 않았습니다.'
      );

      setRobotStatus('idle');
    }
  };

  // =========================
  // ESP32 연결
  // =========================

  const connectESP32 = async () => {
    // 에뮬레이터
    if (TEST_MODE) {
      setConnectionStatus('connecting');

      setTimeout(() => {
        setConnectionStatus('connected');

        console.log(
          'TEST MODE CONNECTED'
        );
      }, 500);

      return;
    }

    const permission =
      await requestPermissions();

    if (!permission) {
      setConnectionStatus(
        'disconnected'
      );

      return;
    }

    setConnectionStatus('connecting');

    manager.startDeviceScan(
      null,
      null,
      async (
        error,
        scannedDevice
      ) => {
        if (error) {
          console.log(error);

          manager.stopDeviceScan();

          setConnectionStatus(
            'disconnected'
          );

          return;
        }

        if (
          scannedDevice?.name ===
          BLE_DEVICE_NAME
        ) {
          manager.stopDeviceScan();

          try {
            const connectedDevice =
              await scannedDevice.connect();

            await connectedDevice
              .discoverAllServicesAndCharacteristics();

            // 기존 Notify 구독이 있으면 제거
            notifySubscriptionRef.current?.remove();

            // ESP32 -> App Notify 구독
            notifySubscriptionRef.current =
              connectedDevice
                .monitorCharacteristicForService(
                  SERVICE_UUID,
                  TX_UUID,
                  (
                    notifyError,
                    characteristic
                  ) => {
                    if (notifyError) {
                      console.log(
                        'BLE NOTIFY ERROR:',
                        notifyError
                      );

                      return;
                    }

                    if (
                      !characteristic?.value
                    ) {
                      return;
                    }

                    try {
                      const decoded =
                        atob(
                          characteristic.value
                        );

                      handleEsp32Message(
                        decoded
                      );
                    } catch (
                      decodeError
                    ) {
                      console.log(
                        'BLE NOTIFY DECODE ERROR:',
                        decodeError
                      );
                    }
                  }
                );

            setDevice(
              connectedDevice
            );

            setConnectionStatus(
              'connected'
            );

            console.log(
              'ESP32 CONNECTED'
            );

            manager.onDeviceDisconnected(
              connectedDevice.id,
              () => {
                notifySubscriptionRef.current?.remove();
                notifySubscriptionRef.current = null;

                setDevice(null);

                setConnectionStatus(
                  'disconnected'
                );

                setRobotStatus(
                  'idle'
                );
              }
            );
          } catch (error) {
            console.log(error);

            setConnectionStatus(
              'disconnected'
            );
          }
        }
      }
    );
  };

  // =========================
  // 연결 해제
  // =========================

  const disconnectESP32 = async () => {
    notifySubscriptionRef.current?.remove();
    notifySubscriptionRef.current = null;

    if (TEST_MODE) {
      setConnectionStatus(
        'disconnected'
      );

      setRobotStatus('idle');

      segmentOffsetRef.current = 0;
      updateCurrentDistance(0);

      updateActiveTargetDistance(
        targetDistance
      );

      return;
    }

    if (!device) {
      return;
    }

    try {
      await device.cancelConnection();

      setDevice(null);

      setConnectionStatus(
        'disconnected'
      );

      setRobotStatus('idle');

      segmentOffsetRef.current = 0;
      updateCurrentDistance(0);

      updateActiveTargetDistance(
        targetDistance
      );
    } catch (error) {
      console.log(error);
    }
  };

  // =========================
  // BLE 명령 전송
  // =========================

  const sendCommand = async (
    command: string
  ) => {
    if (TEST_MODE) {
      console.log(
        'TEST SEND:',
        command
      );

      return true;
    }

    if (!device) {
      return false;
    }

    try {
      const base64Value =
        btoa(command);

      await device
        .writeCharacteristicWithResponseForService(
          SERVICE_UUID,
          RX_UUID,
          base64Value
        );

      console.log(
        'SEND:',
        command
      );

      return true;
    } catch (error) {
      console.log(error);

      return false;
    }
  };

  // =========================
  // START
  // =========================

  const startDriving = async () => {
    const connected =
      TEST_MODE ||
      connectionStatus ===
        'connected';

    if (!connected) {
      return;
    }

    if (targetDistance <= 0) {
      return;
    }

    let distanceToSend = 0;

    // =====================================
    // STOP 후 START
    // 기존 주행 이어가기
    // =====================================
    if (robotStatus === 'stopped') {
      const remainingDistance =
        activeTargetDistanceRef.current -
        currentDistanceRef.current;

      if (remainingDistance <= 0) {
        setRobotStatus('arrived');
        return;
      }

      // ESP32는 새 START 기준으로 거리를 다시 0부터 보내므로
      // 앱 표시에는 기존 이동거리를 offset으로 더한다.
      segmentOffsetRef.current =
        currentDistanceRef.current;

      distanceToSend =
        remainingDistance;
    }

    // =====================================
    // 처음 START / 도착 후 새 START
    // 완전히 새로운 주행
    // =====================================
    else {
      distanceToSend =
        targetDistance;

      updateActiveTargetDistance(
        targetDistance
      );

      segmentOffsetRef.current = 0;
      updateCurrentDistance(0);
    }

    if (distanceToSend <= 0) {
      return;
    }

    // 이번 구간의 목표 거리 전송
    const distanceSent =
      await sendCommand(
        `TARGET:${distanceToSend}`
      );

    if (!distanceSent) {
      return;
    }

    const startSent =
      await sendCommand('START');

    if (!startSent) {
      return;
    }

    // 실제 상태는 ESP32의 DRIVING Notify로도 갱신되지만
    // 버튼 반응을 즉시 보여주기 위해 여기서도 갱신한다.
    setRobotStatus('driving');
  };

  // =========================
  // STOP
  // =========================

  const stopDriving = async () => {
    const stopSent =
      await sendCommand('STOP');

    if (stopSent) {
      // 현재 엔코더 거리는 그대로 유지
      setRobotStatus(
        'stopped'
      );
    }

    console.log(
      'STOP AT:',
      currentDistanceRef.current
    );
  };

  // =========================
  // RESET
  // =========================

  const resetDistance = async () => {
    // 외부 ESP32 코드와 명령 문자열 일치
    const resetSent =
      await sendCommand(
        'RESET_DISTANCE'
      );

    if (!resetSent) {
      return;
    }

    segmentOffsetRef.current = 0;
    updateCurrentDistance(0);

    setRobotStatus('idle');

    // 현재 입력창의 목표거리로 새 주행 준비
    updateActiveTargetDistance(
      targetDistance
    );

    console.log('RESET');
  };

  // =========================
  // 화면
  // =========================

  return (
    <RobotControlScreen
      connectionStatus={
        connectionStatus
      }

      hc12Status={undefined}

      robotStatus={
        robotStatus
      }

      // 입력창용 목표거리
      targetDistance={
        targetDistance
      }

      // 진행률 카드용 전체 목표거리
      activeTargetDistance={
        activeTargetDistance
      }

      // 실제 엔코더 기반 현재거리
      currentDistance={
        currentDistance
      }

      deviceName={
        TEST_MODE
          ? `${BLE_DEVICE_NAME} (TEST)`
          : device?.name ??
            BLE_DEVICE_NAME
      }

      warnings={{}}

      onConnect={
        connectESP32
      }

      onDisconnect={
        disconnectESP32
      }

      onTargetDistanceChange={
        changeTargetDistance
      }

      onStart={
        startDriving
      }

      onStop={
        stopDriving
      }

      onResetDistance={
        resetDistance
      }
    />
  );
}
