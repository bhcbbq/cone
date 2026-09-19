import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { ConnectionStatus, RobotStatus } from '../types/robotUi';
import { colors, typography, spacing, radius, shadow, connectionColor, statusColor } from '../theme/theme';

const CONN_LABELS: Record<ConnectionStatus, string> = {
  connected: '정상',
  connecting: '연결 중',
  disconnected: '끊김',
};

const ROBOT_LABELS: Record<RobotStatus, string> = {
  idle: '대기',
  driving: '주행 중',
  stopped: '정지',
  arrived: '도착 완료',
  returning: '복귀 중',
  returnCompleted: '복귀 완료',
};

interface Row {
  key: string;
  icon: keyof typeof Ionicons.glyphMap;
  label: string;
  value: string;
  color: string;
}

interface Props {
  bleStatus: ConnectionStatus;
  /** HC-12 미연동 시 생략 가능 — "미사용" 항목으로 흐리게 표시됩니다 */
  hc12Status?: ConnectionStatus;
  robotStatus: RobotStatus;
}

/**
 * 장치/오류 상태 패널.
 * 지금은 BLE, HC-12, 로봇 상태 3가지만 표시하지만,
 * 아래 rows 배열에 항목을 추가하는 것만으로 장애물 감지, 전도 감지 등
 * 개별 상태 항목을 손쉽게 확장할 수 있는 구조입니다.
 */
export function DeviceStatusPanel({ bleStatus, hc12Status, robotStatus }: Props) {
  const rows: Row[] = [
    {
      key: 'ble',
      icon: 'bluetooth',
      label: 'BLE 통신',
      value: CONN_LABELS[bleStatus],
      color: connectionColor(bleStatus),
    },
    {
      key: 'hc12',
      icon: 'radio',
      label: 'HC-12 무선',
      value: hc12Status ? CONN_LABELS[hc12Status] : '미사용',
      color: hc12Status ? connectionColor(hc12Status) : colors.textMuted,
    },
    {
      key: 'robot',
      icon: 'navigate',
      label: '로봇 주행 상태',
      value: ROBOT_LABELS[robotStatus],
      color: statusColor(robotStatus),
    },
  ];

  return (
    <View style={[styles.card, shadow.card]}>
      <Text style={styles.title}>장치 상태</Text>
      <View style={styles.rows}>
        {rows.map((row) => (
          <View key={row.key} style={styles.row}>
            <View style={styles.rowLeft}>
              <Ionicons name={row.icon} size={16} color={colors.textSecondary} style={{ marginRight: 8 }} />
              <Text style={styles.rowLabel}>{row.label}</Text>
            </View>
            <View style={[styles.valuePill, { backgroundColor: row.color + '1A' }]}>
              <View style={[styles.valueDot, { backgroundColor: row.color }]} />
              <Text style={[styles.valueText, { color: row.color }]}>{row.value}</Text>
            </View>
          </View>
        ))}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  card: {
    backgroundColor: colors.surface,
    borderRadius: radius.lg,
    borderWidth: 1,
    borderColor: colors.border,
    padding: spacing.md,
  },
  title: {
    ...typography.eyebrow,
    color: colors.textMuted,
    marginBottom: spacing.sm,
  },
  rows: {
    marginTop: 2,
  },
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingVertical: 8,
    borderTopWidth: 1,
    borderTopColor: colors.border,
  },
  rowLeft: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  rowLabel: {
    ...typography.body,
    color: colors.textPrimary,
    fontSize: 13,
  },
  valuePill: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingVertical: 4,
    paddingHorizontal: 10,
    borderRadius: radius.pill,
  },
  valueDot: {
    width: 6,
    height: 6,
    borderRadius: 3,
    marginRight: 6,
  },
  valueText: {
    ...typography.caption,
    fontSize: 11,
  },
});
