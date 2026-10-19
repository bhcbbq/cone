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

                {/* 연결 / 연결 해제 */}

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

              {/* BLE / HC-12 상태 */}

              <View
                style={
                  styles.statusBadgeRow
                }
              >
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

            {/* =========================
                주행 속도
                - 목표거리 바로 아래
                - 주행 중에도 조절 가능
            ========================= */}

            <View style={styles.section}>
              <View style={styles.speedCard}>
                <View style={styles.speedHeader}>
                  <View style={styles.speedTextGroup}>
                    <Text style={styles.speedTitle}>
                      주행 속도
                    </Text>
                    <Text style={styles.speedDescription}>
                      주행 중에도 조절할 수 있습니다
                    </Text>
                  </View>

                  <Text style={styles.speedValue}>
                    {Math.round(currentSpeed * 100)}%
                  </Text>
                </View>

                <View
                  style={styles.sliderTouchArea}
                  onTouchStart={() => setIsSliding(true)}
                  onTouchEnd={() => setIsSliding(false)}
                  onTouchCancel={() => setIsSliding(false)}
                >
                  <Slider
                    style={styles.slider}
                    minimumValue={0.1}
                    maximumValue={0.5}
                    step={0.05}
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
                    maximumTrackTintColor={
                      colors.border
                    }
                    thumbImage={sliderThumb}
                  />
                </View>

                <View
                  style={styles.speedRangeRow}
                >
                  <Text
                    style={styles.speedRangeText}
                  >
                    10%
                  </Text>

                  <Text
                    style={styles.speedRangeText}
                  >
                    50%
                  </Text>
                </View>
              </View>
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

            {/* 현재 속도 표시 */}
            <View style={styles.manualSpeedCard}>
              <View>
                <Text style={styles.manualCardLabel}>
                  현재 주행 속도
                </Text>
                <Text style={styles.manualCardHint}>
                  자동 주행 화면의 슬라이더와 동일한 값입니다
                </Text>
              </View>

              <Text style={styles.manualSpeedValue}>
                {Math.round(
                  currentSpeed * 100
                )}
                %
              </Text>
            </View>

            {/* 방향키 */}
            <View style={styles.manualCard}>
              <Text style={styles.manualCardTitle}>
                방향 조작
              </Text>

              <Text style={styles.manualCardHint}>
                현재는 UI만 구성되어 있으며 실제 주행 명령은 다음 단계에서 연결합니다.
              </Text>

              <View style={styles.directionPad}>
                <Pressable
                  style={({ pressed }) => [
                    styles.directionButton,
                    pressed &&
                    styles.manualPressed,
                  ]}
                >
                  <Ionicons
                    name="caret-up"
                    size={34}
                    color={colors.textPrimary}
                  />
                </Pressable>

                <View style={styles.directionMiddleRow}>
                  <Pressable
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-back"
                      size={34}
                      color={colors.textPrimary}
                    />
                  </Pressable>

                  <Pressable
                    style={({ pressed }) => [
                      styles.manualStopButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="stop"
                      size={27}
                      color={colors.white}
                    />
                  </Pressable>

                  <Pressable
                    style={({ pressed }) => [
                      styles.directionButton,
                      pressed &&
                      styles.manualPressed,
                    ]}
                  >
                    <Ionicons
                      name="caret-forward"
                      size={34}
                      color={colors.textPrimary}
                    />
                  </Pressable>
                </View>

                <Pressable
                  style={({ pressed }) => [
                    styles.directionButton,
                    pressed &&
                    styles.manualPressed,
                  ]}
                >
                  <Ionicons
                    name="caret-down"
                    size={34}
                    color={colors.textPrimary}
                  />
                </Pressable>
              </View>
            </View>

            <View style={styles.manualNotice}>
              <Ionicons
                name="information-circle-outline"
                size={18}
                color={colors.textMuted}
              />
              <Text style={styles.manualNoticeText}>
                실제 제어 연결 시 버튼을 누르는 동안 이동하고 손을 떼면 즉시 정지하도록 구현할 예정입니다.
              </Text>
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
    marginBottom:
      spacing.lg,
  },

  headerTop: {
    flexDirection: 'row',

    justifyContent:
      'space-between',

    alignItems: 'center',
  },

  headerText: {
    flex: 1,

    paddingRight:
      spacing.sm,
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
    minHeight: 42,

    paddingHorizontal:
      spacing.md,

    borderRadius:
      radius.pill,

    flexDirection: 'row',

    alignItems: 'center',

    justifyContent:
      'center',
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

  statusBadgeRow: {
    flexDirection: 'row',

    marginTop:
      spacing.md,

    flexWrap: 'wrap',
  },

  statusBadge: {
    flexDirection: 'row',

    alignItems: 'center',

    borderRadius:
      radius.pill,

    paddingVertical: 8,

    paddingHorizontal: 12,

    marginRight:
      spacing.sm,

    marginBottom:
      spacing.xs,
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
    paddingTop: 20,
    paddingBottom: 18,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 3 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 3,
  },

  speedHeader: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    justifyContent: 'space-between',
    marginBottom: 12,
  },

  speedTextGroup: {
    flex: 1,
    paddingRight: 12,
  },

  speedTitle: {
    ...typography.h2,
    color: colors.textMuted,
    fontSize: 14,
    fontWeight: '700',
  },

  speedDescription: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 11,
    marginTop: 5,
    lineHeight: 16,
  },

  speedValue: {
    ...typography.h1,
    color: colors.accent,
    fontSize: 20,
    fontWeight: '800',
    lineHeight: 26,
  },

  sliderTouchArea: {
    width: '100%',
    paddingVertical: 4,
  },

  slider: {
    width: '100%',
    height: 44,
  },

  speedRangeRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 2,
    marginTop: 0,
  },

  speedRangeText: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 11,
    lineHeight: 15,
    fontWeight: '600',
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

  manualSpeedCard: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    borderWidth: 1,
    borderColor: colors.border,
    borderRadius: 24,
    paddingHorizontal: 22,
    paddingVertical: 20,
    marginBottom: spacing.md,
    backgroundColor: colors.white,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 3 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 3,
  },

  manualCardLabel: {
    ...typography.h2,
    color: colors.textMuted,
    fontSize: 14,
    fontWeight: '700',
  },

  manualCardHint: {
    ...typography.caption,
    color: colors.textMuted,
    fontSize: 11,
    marginTop: 5,
    lineHeight: 16,
  },

  manualSpeedValue: {
    ...typography.h1,
    color: colors.accent,
    fontSize: 20,
    fontWeight: '800',
    marginLeft: spacing.md,
  },

  manualCard: {
    borderWidth: 1,
    borderColor: colors.border,
    borderRadius: 24,
    paddingHorizontal: 22,
    paddingTop: 20,
    paddingBottom: 24,
    backgroundColor: colors.white,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 3 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 3,
  },

  manualCardTitle: {
    ...typography.h2,
    color: colors.textMuted,
    fontSize: 14,
    fontWeight: '700',
  },

  directionPad: {
    alignItems: 'center',
    marginTop: spacing.lg,
  },

  directionMiddleRow: {
    flexDirection: 'row',
    alignItems: 'center',
    marginVertical: 12,
  },

  directionButton: {
    width: 84,
    height: 72,
    borderRadius: 18,
    backgroundColor: colors.surfaceMuted,
    borderWidth: 1,
    borderColor: colors.border,
    alignItems: 'center',
    justifyContent: 'center',
    marginHorizontal: 7,
  },

  manualStopButton: {
    width: 84,
    height: 72,
    borderRadius: 18,
    backgroundColor: colors.danger,
    alignItems: 'center',
    justifyContent: 'center',
    marginHorizontal: 7,
  },

  manualPressed: {
    opacity: 0.72,
    transform: [{ scale: 0.97 }],
  },

  manualNotice: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    marginTop: spacing.md,
    paddingHorizontal: 18,
    paddingVertical: 14,
    borderRadius: 18,
    backgroundColor: colors.surfaceMuted,
    borderWidth: 1,
    borderColor: colors.border,
  },

  manualNoticeText: {
    ...typography.caption,
    flex: 1,
    color: colors.textMuted,
    fontSize: 11,
    lineHeight: 17,
    marginLeft: 7,
  },
});
