import React, { useEffect, useState } from 'react';

import {
  ActivityIndicator,
  Pressable,
  SafeAreaView,
  ScrollView,
  StatusBar,
  StyleSheet,
  Text,
  useWindowDimensions,
  View,
} from 'react-native';

import { Ionicons } from '@expo/vector-icons';
import Slider from '@react-native-community/slider';

const sliderThumb = require('../../assets/images/slider-thumb-square.png');

// 실제 UGV에 적용할 속도 범위
const SPEED_MIN = 0.2;
const SPEED_MAX = 0.5;
const SPEED_STEP = 0.05;

// 앱에 표시할 속도 범위
// 실제 0.2 ~ 0.5 값을 UI에서는 10% ~ 100%로 표시합니다.
const DISPLAY_SPEED_MIN = 10;
const DISPLAY_SPEED_MAX = 100;

import {
  ConnectionStatus,
  RobotStatus,
  WarningFlags,
} from '../types/robotUi';

import {
  colors,
  connectionColor,
  radius,
  spacing,
  typography,
} from '../theme/theme';

import { ArrivalBanner } from '../components/ArrivalBanner';
import { ControlButtons } from '../components/ControlButtons';
import { DistanceInputCard } from '../components/DistanceInputCard';
import { ProgressCard } from '../components/ProgressCard';
import { WarningBanner } from '../components/WarningBanner';

type ManualDirection =
  | 'forward'
  | 'forwardLeft'
  | 'forwardRight'
  | 'left'
  | 'right'
  | 'backward'
  | 'backwardLeft'
  | 'backwardRight'
  | 'stop';

interface Props {
  connectionStatus: ConnectionStatus;

  hc12Status?: ConnectionStatus;

  robotStatus: RobotStatus;

  // 다음 주행에 입력할 목표거리
  targetDistance: number;

  // 현재 또는 직전 주행에서 실제 사용한 목표거리
  activeTargetDistance: number;

  currentDistance: number;

  warnings?: WarningFlags;

  deviceName?: string;

  onConnect: () => void;

  onDisconnect: () => void;

  onTargetDistanceChange: (
    value: number
  ) => void;

  onStart: () => void;

  onStop: () => void;

  onReturn?: () => void;

  onResetDistance?: () => void;

  // 나중에 실제 속도 제어를 연결할 때 사용
  speed?: number;

  // 슬라이더 이동 중 UI 값 반영용
  onSpeedChange?: (value: number) => void;

  // 슬라이더에서 손을 뗐을 때 실제 명령 전송용
  onSpeedChangeComplete?: (value: number) => void;

  // UGV에서 수신한 배터리 전압. 아직 연결하지 않으면 --.- V 표시
  batteryVoltage?: number;

  // 실제 배터리 잔량(0~100)을 받을 수 있으면 이 값을 넘기면 됩니다.
  // 넘기지 않았을 때는 3S 배터리 전압으로 대략적인 잔량을 계산합니다.
  batteryPercent?: number;

  // 이후 실제 수동 주행 명령 연결용 (지금은 넘기지 않아도 UI만 동작)
  onManualPress?: (direction: ManualDirection) => void;
  onManualRelease?: () => void;
}

export function RobotControlScreen({
  connectionStatus,
  hc12Status,
  robotStatus,

  targetDistance,
  activeTargetDistance,
  currentDistance,

  warnings,
  deviceName,

  onConnect,
  onDisconnect,

  onTargetDistanceChange,

  onStart,
  onStop,
  onReturn,
  onResetDistance,

  speed,
  onSpeedChange,
  onSpeedChangeComplete,

  batteryVoltage,
  batteryPercent,
  onManualPress,
  onManualRelease,
}: Props) {
  const { width: screenWidth } = useWindowDimensions();

  // speed prop을 아직 index.tsx에서 넘기지 않아도 UI 확인 가능하도록
  // 내부 기본값 30% 사용
  const [localSpeed, setLocalSpeed] = useState(
    speed ?? 0.3
  );

  const [isSliding, setIsSliding] = useState(false);

  useEffect(() => {
    if (typeof speed === 'number') {
      setLocalSpeed(speed);
    }
  }, [speed]);

  const currentSpeed = localSpeed;

  const displaySpeedPercent = (() => {
    const normalized =
      (currentSpeed - SPEED_MIN) /
      (SPEED_MAX - SPEED_MIN);

    const displayValue =
      DISPLAY_SPEED_MIN +
      normalized *
      (DISPLAY_SPEED_MAX - DISPLAY_SPEED_MIN);

    return Math.max(
      DISPLAY_SPEED_MIN,
      Math.min(
        DISPLAY_SPEED_MAX,
        Math.round(displayValue)
      )
    );
  })();

  const isBusy =
    robotStatus === 'driving' ||
    robotStatus === 'returning';

  const isConnected =
    connectionStatus === 'connected';

  const isConnecting =
    connectionStatus === 'connecting';

  const bleColor =
    connectionColor(connectionStatus);

  const hc12Connected =
    hc12Status === 'connected';

  const hc12Connecting =
    hc12Status === 'connecting';

  const estimatedBatteryPercent = (() => {
    if (typeof batteryPercent === 'number') {
      return Math.max(
        0,
        Math.min(100, Math.round(batteryPercent))
      );
    }

    if (typeof batteryVoltage !== 'number') {
      return undefined;
    }

    // 3S 배터리 기준 UI용 대략적인 잔량 추정.
    // 실제 BMS/UGV에서 % 값을 받을 수 있으면 batteryPercent를 우선 사용합니다.
    const BATTERY_EMPTY_V = 9.6;
    const BATTERY_FULL_V = 12.6;

    const percent =
      ((batteryVoltage - BATTERY_EMPTY_V) /
        (BATTERY_FULL_V - BATTERY_EMPTY_V)) *
      100;

    return Math.max(
      0,
      Math.min(100, Math.round(percent))
    );
  })();

  const handleSpeedChange = (
    value: number
  ) => {
    setLocalSpeed(value);
    onSpeedChange?.(value);
  };

  const handleSpeedChangeComplete = (
    value: number
  ) => {
    setLocalSpeed(value);
    onSpeedChangeComplete?.(value);
  };

  const handleManualPress = (
    direction: ManualDirection
  ) => {
    onManualPress?.(direction);
  };

  const handleManualRelease = () => {
    onManualRelease?.();
  };

  return (
    <SafeAreaView style={styles.safe}>
      <StatusBar
        barStyle="dark-content"
        backgroundColor={
          colors.background
        }
      />

      {/*
        좌우 페이지
        1페이지: 기존 자동 주행 화면
        2페이지: 수동 조작 화면
      */}
      <ScrollView
        horizontal
        pagingEnabled
        scrollEnabled={!isSliding}
        showsHorizontalScrollIndicator={
          false
        }
        style={styles.pager}
      >
        {/* =========================
            1PAGE - 자동 주행
        ========================= */}
        <View
          style={[
            styles.page,
            { width: screenWidth },
          ]}
        >
          <ScrollView
            style={styles.scroll}
            contentContainerStyle={
              styles.container
            }
            showsVerticalScrollIndicator={
              false
            }
            keyboardShouldPersistTaps="handled"
          >
            {/* =========================
                상단 헤더
            ========================= */}

            <View style={styles.header}>
              <View style={styles.headerTop}>
                <View style={styles.headerText}>
                  <Text style={styles.eyebrow}>
                    SMART LAVA-CONE UGV
                  </Text>

                  <Text style={styles.title}>
                    주행 관리
                  </Text>

                  <Text style={styles.deviceName}>
                    {deviceName ??
                      'ESP32_CONE'}
                  </Text>
                </View>

                {/* 연결 */}

                <View style={styles.headerRight}>
                  <Pressable
                    onPress={
                      isConnected
                        ? onDisconnect
                        : onConnect
                    }
                    disabled={isConnecting}
                    style={({ pressed }) => [
                      styles.connectionButton,

                      isConnected
                        ? styles.disconnectButton
                        : styles.connectButton,

                      isConnecting &&
                      styles.connectionButtonDisabled,

                      pressed &&
                      !isConnecting && {
                        opacity: 0.8,
                      },
                    ]}
                  >
                    {isConnecting ? (
                      <ActivityIndicator
                        size="small"
                        color={colors.white}
                      />
                    ) : (
                      <Ionicons
                        name={
                          isConnected
                            ? 'close-circle-outline'
                            : 'bluetooth'
                        }
                        size={17}
                        color={
                          isConnected
                            ? colors.danger
                            : colors.white
                        }
                      />
                    )}

                    <Text
                      style={[
                        styles.connectionButtonText,

                        isConnected
                          ? styles.disconnectText
                          : styles.connectText,
                      ]}
                    >
                      {isConnecting
                        ? '연결 중'
                        : isConnected
                          ? '연결 해제'
                          : '연결하기'}
                    </Text>
                  </Pressable>
                </View>
              </View>

              {/* 통신 상태 + 배터리 */}
              <View style={styles.headerInfoRow}>
                <View style={styles.statusGroup}>
                  <View
                    style={[
                      styles.statusBadge,
                      {
                        backgroundColor:
                          isConnected
                            ? colors.statusDrivingSoft
                            : isConnecting
                              ? colors.statusArrivedSoft
                              : colors.statusStoppedSoft,
                      },
                    ]}
                  >
                    <Ionicons
                      name="bluetooth"
                      size={17}
                      color={bleColor}
                    />

                    <Text
                      style={[
                        styles.statusBadgeText,
                        {
                          color: bleColor,
                        },
                      ]}
                    >
                      {isConnected
                        ? 'BLE 연결'
                        : isConnecting
                          ? 'BLE 연결 중'
                          : 'BLE 미연결'}
                    </Text>
                  </View>

                  <View
                    style={[
                      styles.statusBadge,
                      {
                        backgroundColor:
                          hc12Connected
                            ? colors.statusDrivingSoft
                            : hc12Connecting
                              ? colors.statusArrivedSoft
                              : colors.surfaceMuted,
                      },
                    ]}
                  >
                    <Ionicons
                      name="radio-outline"
                      size={17}
                      color={
                        hc12Connected
                          ? colors.success
                          : hc12Connecting
                            ? colors.warning
                            : colors.textMuted
                      }
                    />

                    <Text
                      style={[
                        styles.statusBadgeText,
                        {
                          color:
                            hc12Connected
                              ? colors.success
                              : hc12Connecting
                                ? colors.warning
                                : colors.textMuted,
                        },
                      ]}
                    >
                      {hc12Connected
                        ? 'HC-12 연결'
                        : hc12Connecting
                          ? 'HC-12 연결 중'
                          : 'HC-12 미연결'}
                    </Text>
                  </View>
                </View>

                <View style={styles.headerBatteryCard}>
                  <Ionicons
                    name="battery-half-outline"
                    size={18}
                    color={colors.accent}
                  />

                  <View style={styles.headerBatteryContent}>
                    <View style={styles.headerBatteryTopRow}>
                      <View>
                        <Text style={styles.headerBatteryLabel}>
                          배터리
                        </Text>

                        <Text style={styles.headerBatteryValue}>
                          {typeof batteryVoltage === 'number'
                            ? `${batteryVoltage.toFixed(1)} V`
                            : '--.- V'}
                        </Text>
                      </View>

                      <Text style={styles.headerBatteryPercent}>
                        {typeof estimatedBatteryPercent === 'number'
                          ? `${estimatedBatteryPercent}%`
                          : '--%'}
                      </Text>
                    </View>

                    <View style={styles.headerBatteryGauge}>
                      <View
                        style={[
                          styles.headerBatteryGaugeFill,
                          {
                            width:
                              typeof estimatedBatteryPercent === 'number'
                                ? `${estimatedBatteryPercent}%`
                                : '0%',
                          },
                        ]}
                      />
                    </View>
                  </View>
                </View>
              </View>
            </View>

            {/* 페이지 안내 */}
            <View style={styles.pageGuide}>
              <View style={styles.pageDots}>
                <View
                  style={[
                    styles.pageDot,
                    styles.pageDotActive,
                  ]}
                />
                <View style={styles.pageDot} />
              </View>

              <View style={styles.swipeHint}>
                <Text style={styles.swipeHintText}>
                  수동 조작
                </Text>
                <Ionicons
                  name="chevron-forward"
                  size={16}
                  color={colors.textMuted}
                />
              </View>
            </View>

            {/* =========================
                도착 완료 알림
            ========================= */}

            <ArrivalBanner
              robotStatus={robotStatus}
              targetDistance={
                activeTargetDistance
              }
            />

            {/* =========================
                경고
            ========================= */}

            <WarningBanner
              warnings={warnings}
            />

            {/* =========================
                진행률
            ========================= */}

            <View style={styles.section}>
              <ProgressCard
                robotStatus={robotStatus}
                targetDistance={
                  activeTargetDistance
                }
                currentDistance={
                  currentDistance
                }
              />
            </View>

            {/* =========================
                목표거리
            ========================= */}

            <View style={styles.section}>
              <DistanceInputCard
                value={targetDistance}
                editable={!isBusy}
                onChange={
                  onTargetDistanceChange
                }
              />
            </View>

            {/*
              하단 고정 컨트롤에 가리지 않도록
              마지막에 공간 확보
            */}

            <View
              style={styles.bottomSpacer}
            />
          </ScrollView>

          {/* =========================
              하단 고정 컨트롤
          ========================= */}

          <View
            style={styles.bottomControls}
          >
            <ControlButtons
              robotStatus={robotStatus}
              targetDistance={
                targetDistance
              }
              isBleConnected={
                connectionStatus ===
                'connected'
              }
              onStart={onStart}
              onStop={onStop}
              onReturn={onReturn}
              onResetDistance={
                onResetDistance
              }
            />
          </View>
        </View>

        {/* =========================
            2PAGE - 수동 조작
        ========================= */}
        <View
          style={[
            styles.page,
            { width: screenWidth },
          ]}
        >
          <ScrollView
            style={styles.scroll}
            contentContainerStyle={
              styles.manualContainer
            }
            showsVerticalScrollIndicator={
              false
            }
          >
            <View style={styles.manualHeader}>
              <View>
                <Text style={styles.eyebrow}>
                  SMART LAVA-CONE UGV
                </Text>

                <Text style={styles.title}>
                  수동 조작
                </Text>

                <Text style={styles.deviceName}>
                  {deviceName ??
                    'ESP32_CONE'}
                </Text>
              </View>

              <Ionicons
                name="game-controller-outline"
                size={30}
                color={colors.accent}
              />
            </View>

            <View style={styles.pageGuide}>
              <View style={styles.pageDots}>
                <View style={styles.pageDot} />
                <View
                  style={[
                    styles.pageDot,
                    styles.pageDotActive,
                  ]}
                />
              </View>

              <View style={styles.swipeHint}>
                <Ionicons
                  name="chevron-back"
                  size={16}
                  color={colors.textMuted}
                />
                <Text style={styles.swipeHintText}>
                  자동 주행
                </Text>
              </View>
            </View>

            {/* 연결 상태 간단 표시 */}
            <View style={styles.manualStatusRow}>
              <View
                style={[
                  styles.statusBadge,
                  {
                    backgroundColor:
                      isConnected
                        ? colors.statusDrivingSoft
                        : colors.statusStoppedSoft,
                  },
                ]}
              >
                <Ionicons
                  name="bluetooth"
                  size={17}
                  color={bleColor}
                />

                <Text
                  style={[
                    styles.statusBadgeText,
                    { color: bleColor },
                  ]}
                >
                  {isConnected
                    ? 'BLE 연결'
                    : 'BLE 미연결'}
                </Text>
              </View>

              <View
                style={[
                  styles.statusBadge,
                  {
                    backgroundColor:
                      hc12Connected
                        ? colors.statusDrivingSoft
                        : colors.surfaceMuted,
                  },
                ]}
              >
                <Ionicons
                  name="radio-outline"
                  size={17}
                  color={
                    hc12Connected
                      ? colors.success
                      : colors.textMuted
                  }
                />

                <Text
                  style={[
                    styles.statusBadgeText,
                    {
                      color: hc12Connected
                        ? colors.success
                        : colors.textMuted,
                    },
                  ]}
                >
                  {hc12Connected
                    ? 'HC-12 연결'
                    : 'HC-12 미연결'}
                </Text>
              </View>
            </View>

            {/* 수동 주행 속도 */}
            <View style={styles.section}>
              <View style={styles.speedCard}>
                <View style={styles.speedHeader}>
                  <Text style={styles.cardLabel}>
                    주행 속도
                  </Text>

                  <Text style={styles.speedValue}>
                    {displaySpeedPercent}%
                  </Text>
                </View>

                <View style={styles.speedSliderBox}>
                  <View
                    style={styles.sliderTouchArea}
                    onTouchStart={() => setIsSliding(true)}
                    onTouchEnd={() => setIsSliding(false)}
                    onTouchCancel={() => setIsSliding(false)}
                  >
                    {/* Android 기본 maximum track이 너무 연하게 보여서
                        전체 슬라이더 길이가 확실히 보이도록 레일을 하나 깔아둡니다. */}
                    <View style={styles.sliderFullTrack} />

                    <Slider
                      style={styles.slider}
                      minimumValue={SPEED_MIN}
                      maximumValue={SPEED_MAX}
                      step={SPEED_STEP}
                      value={currentSpeed}
                      onValueChange={
                        handleSpeedChange
                      }
                      onSlidingStart={() => {
                        setIsSliding(true);
                      }}
                      onSlidingComplete={(value) => {
                        setIsSliding(false);
                        handleSpeedChangeComplete(value);
                      }}
                      minimumTrackTintColor={
                        colors.accent
                      }
                      maximumTrackTintColor="transparent"
                      thumbImage={sliderThumb}
                    />
                  </View>

                  <View style={styles.speedRangeRow}>
                    <Text style={styles.speedRangeText}>
                      SLOW
                    </Text>

                    <Text style={styles.speedRangeText}>
                      FAST
                    </Text>
                  </View>
                </View>
              </View>
            </View>

            {/* 8방향 수동 조작 */}
            <View style={styles.manualCard}>
              <Text style={styles.cardLabel}>
                방향 조작
              </Text>

              <View style={styles.directionGrid}>
                {/* 1행: ↖ ↑ ↗ */}
                <View style={styles.directionRow}>
                  <Pressable
                    onPressIn={() =>
                      handleManualPress('forwardLeft')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-up"
                      size={29}
                      color={colors.textMuted}
                      style={{
                        opacity: 0.58,
                        transform: [
                          { rotate: '-45deg' },
                        ],
                      }}
                    />
                  </Pressable>

                  <Pressable
                    onPressIn={() =>
                      handleManualPress('forward')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-up"
                      size={31}
                      color={colors.textPrimary}
                    />
                  </Pressable>

                  <Pressable
                    onPressIn={() =>
                      handleManualPress('forwardRight')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-up"
                      size={29}
                      color={colors.textMuted}
                      style={{
                        opacity: 0.58,
                        transform: [
                          { rotate: '45deg' },
                        ],
                      }}
                    />
                  </Pressable>
                </View>

                {/* 2행: ← STOP → */}
                <View style={styles.directionRow}>
                  <Pressable
                    onPressIn={() =>
                      handleManualPress('left')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-back"
                      size={31}
                      color={colors.textPrimary}
                    />
                  </Pressable>

                  <Pressable
                    onPressIn={() =>
                      handleManualPress('stop')
                    }
                    style={({ pressed }) => [
                      styles.manualStopButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="stop"
                      size={25}
                      color={colors.white}
                    />
                  </Pressable>

                  <Pressable
                    onPressIn={() =>
                      handleManualPress('right')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-forward"
                      size={31}
                      color={colors.textPrimary}
                    />
                  </Pressable>
                </View>

                {/* 3행: ↙ ↓ ↘ */}
                <View style={styles.directionRow}>
                  <Pressable
                    onPressIn={() =>
                      handleManualPress('backwardLeft')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-down"
                      size={29}
                      color={colors.textMuted}
                      style={{
                        opacity: 0.58,
                        transform: [
                          { rotate: '45deg' },
                        ],
                      }}
                    />
                  </Pressable>

                  <Pressable
                    onPressIn={() =>
                      handleManualPress('backward')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-down"
                      size={31}
                      color={colors.textPrimary}
                    />
                  </Pressable>

                  <Pressable
                    onPressIn={() =>
                      handleManualPress('backwardRight')
                    }
                    onPressOut={handleManualRelease}
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-down"
                      size={29}
                      color={colors.textMuted}
                      style={{
                        opacity: 0.58,
                        transform: [
                          { rotate: '-45deg' },
                        ],
                      }}
                    />
                  </Pressable>
                </View>
              </View>
            </View>
          </ScrollView>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safe: {
    flex: 1,
    backgroundColor:
      colors.background,
  },

  pager: {
    flex: 1,
  },

  page: {
    flex: 1,
    backgroundColor:
      colors.background,
  },

  scroll: {
    flex: 1,
  },

  container: {
    paddingHorizontal:
      spacing.lg,

    paddingTop:
      spacing.md,
  },

  // =========================
  // HEADER
  // =========================

  header: {
    paddingTop: 24,
    marginBottom: spacing.md,
  },

  headerTop: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'flex-start',
  },

  headerText: {
    flex: 1,
    paddingRight: spacing.md,
    paddingTop: 2,
  },

  headerRight: {
    width: 130,
    alignItems: 'stretch',
    justifyContent: 'flex-start',
  },

  headerBatteryCard: {
    width: 180,
    minHeight: 58,
    paddingHorizontal: 10,
    paddingVertical: 8,
    borderRadius: 16,
    borderWidth: 1,
    borderColor: colors.border,
    backgroundColor: colors.white,
    flexDirection: 'row',
    alignItems: 'center',
    shadowColor: '#000',
    shadowOffset: {
      width: 0,
      height: 2,
    },
    shadowOpacity: 0.05,
    shadowRadius: 4,
    elevation: 2,
  },

  headerBatteryContent: {
    flex: 1,
    marginLeft: 7,
  },

  headerBatteryTopRow: {
    flexDirection: 'row',
    alignItems: 'flex-end',
    justifyContent: 'space-between',
  },

  headerBatteryLabel: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 9,
    lineHeight: 12,
    fontWeight: '600',
    includeFontPadding: false,
  },

  headerBatteryValue: {
    ...typography.h2,
    color: colors.textPrimary,
    fontSize: 14,
    lineHeight: 17,
    fontWeight: '800',
    marginTop: 1,
    includeFontPadding: false,
  },

  headerBatteryPercent: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 10,
    lineHeight: 13,
    fontWeight: '700',
    marginBottom: 1,
    includeFontPadding: false,
  },

  headerBatteryGauge: {
    width: '100%',
    height: 4,
    borderRadius: 2,
    backgroundColor: colors.surfaceMuted,
    overflow: 'hidden',
    marginTop: 5,
  },

  headerBatteryGaugeFill: {
    height: '100%',
    borderRadius: 3,
    backgroundColor: colors.accent,
  },

  eyebrow: {
    ...typography.eyebrow,

    color: colors.accent,

    marginBottom: 5,
  },

  title: {
    ...typography.h1,

    fontSize: 25,

    color:
      colors.textPrimary,
  },

  deviceName: {
    ...typography.h2,

    color:
      colors.textMuted,

    marginTop: 7,

    fontSize: 14,
  },

  // =========================
  // CONNECT BUTTON
  // =========================

  connectionButton: {
    width: '100%',
    minHeight: 46,
    paddingHorizontal: 10,
    borderRadius: radius.pill,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
  },

  connectButton: {
    backgroundColor:
      colors.accent,
  },

  disconnectButton: {
    backgroundColor:
      colors.statusStoppedSoft,
  },

  connectionButtonDisabled: {
    opacity: 0.6,
  },

  connectionButtonText: {
    ...typography.h2,

    fontSize: 13,

    marginLeft: 5,
  },

  connectText: {
    color: colors.white,
  },

  disconnectText: {
    color: colors.danger,
  },

  // =========================
  // BLE / HC12 BADGES
  // =========================

  headerInfoRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginTop: spacing.md,
  },

  statusGroup: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    flexWrap: 'wrap',
    paddingRight: spacing.sm,
  },

  statusBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    borderRadius: radius.pill,
    paddingVertical: 8,
    paddingHorizontal: 10,
    marginRight: 7,
    marginBottom: 0,
  },

  statusBadgeText: {
    ...typography.caption,

    fontSize: 12,

    marginLeft: 5,
  },




  // =========================
  // PAGE GUIDE
  // =========================

  pageGuide: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: spacing.md,
  },

  pageDots: {
    flexDirection: 'row',
    alignItems: 'center',
  },

  pageDot: {
    width: 7,
    height: 7,
    borderRadius: 4,
    backgroundColor: colors.border,
    marginRight: 6,
  },

  pageDotActive: {
    width: 18,
    backgroundColor: colors.accent,
  },

  swipeHint: {
    flexDirection: 'row',
    alignItems: 'center',
  },

  swipeHintText: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 12,
  },

  // =========================
  // CONTENT
  // =========================

  section: {
    marginBottom:
      spacing.md,
  },

  // 목표 거리 / 주행 속도 / 수동 조작 카드 공통 제목
  cardLabel: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 12,
    lineHeight: 16,
    fontWeight: '700',
    includeFontPadding: false,
  },

  bottomSpacer: {
    height: 170,
  },

  // =========================
  // SPEED CARD
  // 목표 거리 카드와 같은 흰색 + 둥근 사각형 스타일
  // =========================

  speedCard: {
    backgroundColor: colors.white,
    borderWidth: 1,
    borderColor: colors.border,
    borderRadius: 24,
    paddingHorizontal: 22,
    paddingTop: 18,
    paddingBottom: 18,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 3 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 3,
  },

  speedHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 12,
  },




  speedValue: {
    ...typography.h1,
    color: colors.accent,
    fontSize: 20,
    lineHeight: 24,
    fontWeight: '800',
    includeFontPadding: false,
  },

  speedSliderBox: {
    backgroundColor: colors.surfaceMuted,
    borderWidth: 1,
    borderColor: colors.border,
    borderRadius: 18,
    paddingHorizontal: 14,
    paddingTop: 8,
    paddingBottom: 8,
  },

  sliderTouchArea: {
    width: '100%',
    height: 38,
    justifyContent: 'center',
    position: 'relative',
  },

  sliderFullTrack: {
    position: 'absolute',
    left: 8,
    right: 8,
    height: 4,
    borderRadius: 2,
    backgroundColor: '#CBD3DE',
  },

  slider: {
    width: '100%',
    height: 38,
  },

  speedRangeRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 4,
    marginTop: 2,
  },

  speedRangeText: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 14,
    lineHeight: 18,
    fontWeight: '700',
    letterSpacing: 0.4,
    includeFontPadding: false,
  },

  // =========================
  // FIXED BOTTOM CONTROLS
  // =========================

  bottomControls: {
    backgroundColor:
      colors.background,

    paddingHorizontal:
      spacing.lg,

    paddingTop:
      spacing.sm,

    paddingBottom:
      spacing.md,

    borderTopWidth: 1,

    borderTopColor:
      colors.border,
  },

  // =========================
  // MANUAL PAGE
  // 목표 거리 카드와 동일한 흰색 카드 + 둥근 사각형 스타일
  // =========================

  manualContainer: {
    paddingHorizontal: spacing.lg,
    paddingTop: spacing.md + 24,
    paddingBottom: 40,
  },

  manualHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: spacing.lg,
  },

  manualStatusRow: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    marginBottom: spacing.md,
  },







  manualCard: {
    borderWidth: 1,
    borderColor: colors.border,
    borderRadius: 24,
    paddingHorizontal: 22,
    paddingTop: 18,
    paddingBottom: 22,
    backgroundColor: colors.white,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 3 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 3,
  },

  directionGrid: {
    alignItems: 'center',
    marginTop: spacing.lg,
  },

  directionRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: 10,
  },

  directionButton: {
    width: 78,
    height: 68,
    borderRadius: 18,
    backgroundColor: colors.surfaceMuted,
    borderWidth: 1,
    borderColor: colors.border,
    alignItems: 'center',
    justifyContent: 'center',
    marginHorizontal: 5,
  },

  manualStopButton: {
    width: 78,
    height: 68,
    borderRadius: 18,
    backgroundColor: colors.danger,
    alignItems: 'center',
    justifyContent: 'center',
    marginHorizontal: 5,
  },

  manualPressed: {
    opacity: 0.72,
    transform: [{ scale: 0.97 }],
  },

});
