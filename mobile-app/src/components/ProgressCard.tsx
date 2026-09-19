import React from 'react';
import { StyleSheet, Text, View } from 'react-native';
import Svg, { Circle } from 'react-native-svg';

import { RobotStatus } from '../types/robotUi';

import {
    colors,
    radius,
    shadow,
    spacing,
    statusColor,
    typography,
} from '../theme/theme';

import { RobotStatusBadge } from './RobotStatusBadge';

const SIZE = 156;
const STROKE = 13;
const RADIUS = (SIZE - STROKE) / 2;
const CIRCUMFERENCE =
  2 * Math.PI * RADIUS;

// 값이 undefined / NaN이어도 오류 안 나게 처리
function fmt(value: unknown) {
  const n = Number(value);

  if (!Number.isFinite(n)) {
    return '0.0';
  }

  return n.toFixed(1);
}

interface Props {
  robotStatus: RobotStatus;
  targetDistance: number;
  currentDistance: number;
}

export function ProgressCard({
  robotStatus,
  targetDistance,
  currentDistance,
}: Props) {
  // 모든 값 안전하게 숫자로 변환
  const safeTarget =
    Number.isFinite(Number(targetDistance))
      ? Number(targetDistance)
      : 0;

  const safeCurrent =
    Number.isFinite(Number(currentDistance))
      ? Number(currentDistance)
      : 0;

  const remainingDistance = Math.max(
    0,
    safeTarget - safeCurrent
  );

  const progressPercent =
    safeTarget > 0
      ? Math.max(
          0,
          Math.min(
            100,
            (safeCurrent / safeTarget) * 100
          )
        )
      : 0;

  const dashOffset =
    CIRCUMFERENCE *
    (1 - progressPercent / 100);

  const ringColor =
    statusColor(robotStatus);

  return (
    <View style={[styles.card, shadow.card]}>
      <View style={styles.header}>
        <Text style={styles.eyebrow}>
          주행 진행 상황
        </Text>

        <RobotStatusBadge
          status={robotStatus}
          size="small"
        />
      </View>

      <View style={styles.body}>
        <View style={styles.ringWrap}>
          <Svg
            width={SIZE}
            height={SIZE}
          >
            <Circle
              cx={SIZE / 2}
              cy={SIZE / 2}
              r={RADIUS}
              stroke={colors.surfaceMuted}
              strokeWidth={STROKE}
              fill="none"
            />

            <Circle
              cx={SIZE / 2}
              cy={SIZE / 2}
              r={RADIUS}
              stroke={ringColor}
              strokeWidth={STROKE}
              fill="none"
              strokeLinecap="round"
              strokeDasharray={`${CIRCUMFERENCE} ${CIRCUMFERENCE}`}
              strokeDashoffset={
                dashOffset
              }
              rotation={-90}
              origin={`${SIZE / 2}, ${SIZE / 2}`}
            />
          </Svg>

          <View
            style={styles.ringCenter}
          >
            <Text
              style={
                styles.percentText
              }
            >
              {progressPercent.toFixed(
                0
              )}
              %
            </Text>

            <Text
              style={
                styles.percentLabel
              }
            >
              {fmt(safeTarget)}m 중
            </Text>
          </View>
        </View>

        <View style={styles.statCol}>
          <StatRow
            label="현재 이동거리"
            value={`${fmt(
              safeCurrent
            )} m`}
            color={
              colors.textPrimary
            }
          />

          <StatRow
            label="남은 거리"
            value={`${fmt(
              remainingDistance
            )} m`}
            color={colors.accent}
          />

          <StatRow
            label="목표 거리"
            value={`${fmt(
              safeTarget
            )} m`}
            color={
              colors.textSecondary
            }
          />
        </View>
      </View>
    </View>
  );
}

function StatRow({
  label,
  value,
  color,
}: {
  label: string;
  value: string;
  color: string;
}) {
  return (
    <View style={styles.statRow}>
      <Text style={styles.statLabel}>
        {label}
      </Text>

      <Text
        style={[
          styles.statValue,
          { color },
        ]}
      >
        {value}
      </Text>
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

  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: spacing.md,
  },

  eyebrow: {
    ...typography.eyebrow,
    color: colors.textMuted,
  },

  body: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent:
      'space-between',
  },

  ringWrap: {
    width: SIZE,
    height: SIZE,
    alignItems: 'center',
    justifyContent: 'center',
  },

  ringCenter: {
    position: 'absolute',
    alignItems: 'center',
  },

  percentText: {
    ...typography.hudNumber,
    fontSize: 34,
    color: colors.textPrimary,
  },

  percentLabel: {
    ...typography.caption,
    color: colors.textMuted,
    marginTop: 2,
  },

  statCol: {
    flex: 1,
    marginLeft: spacing.md,
  },

  statRow: {
    marginBottom: spacing.md,
  },

  statLabel: {
    ...typography.caption,
    color: colors.textMuted,
    marginBottom: 2,
  },

  statValue: {
    ...typography.h1,
    fontSize: 19,
  },
});