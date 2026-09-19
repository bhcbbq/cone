import React from 'react';

import {
  ActivityIndicator,
  Pressable,
  SafeAreaView,
  ScrollView,
  StatusBar,
  StyleSheet,
  Text,
  View,
} from 'react-native';

import { Ionicons } from '@expo/vector-icons';

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
}: Props) {
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

  return (
    <SafeAreaView style={styles.safe}>
      <StatusBar
        barStyle="dark-content"
        backgroundColor={
          colors.background
        }
      />

      {/* =========================
          스크롤되는 화면 영역
      ========================= */}

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
                주행 제어
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
                  ? 'BLE 연결됨'
                  : isConnecting
                    ? 'BLE 연결 중'
                    : 'BLE 끊김'}
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
                  ? 'HC-12 정상'
                  : hc12Connecting
                    ? 'HC-12 확인 중'
                    : 'HC-12 미확인'}
              </Text>
            </View>
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
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safe: {
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
});