import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { RobotStatus } from '../types/robotUi';
import { colors, typography, spacing, radius, statusColor, statusSoftColor } from '../theme/theme';

const LABELS: Record<RobotStatus, string> = {
  idle: '대기',
  driving: '주행 중',
  stopped: '정지',
  arrived: '도착 완료',
  returning: '복귀 중',
  returnCompleted: '복귀 완료',
};

const ICONS: Record<RobotStatus, keyof typeof Ionicons.glyphMap> = {
  idle: 'pause-circle',
  driving: 'navigate-circle',
  stopped: 'stop-circle',
  arrived: 'checkmark-circle',
  returning: 'return-down-back',
  returnCompleted: 'checkmark-done-circle',
};

export function RobotStatusBadge({ status, size = 'large' }: { status: RobotStatus; size?: 'large' | 'small' }) {
  const color = statusColor(status);
  const soft = statusSoftColor(status);
  const isLarge = size === 'large';

  return (
    <View style={[styles.badge, { backgroundColor: soft }, isLarge ? styles.large : styles.small]}>
      <Ionicons name={ICONS[status]} size={isLarge ? 20 : 15} color={color} />
      <Text style={[styles.text, { color }, isLarge ? styles.textLarge : styles.textSmall]}>
        {LABELS[status]}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  badge: {
    flexDirection: 'row',
    alignItems: 'center',
    alignSelf: 'flex-start',
    borderRadius: radius.pill,
  },
  large: {
    paddingVertical: 9,
    paddingHorizontal: spacing.md,
  },
  small: {
    paddingVertical: 5,
    paddingHorizontal: 10,
  },
  text: {
    ...typography.h2,
    marginLeft: 6,
  },
  textLarge: {
    fontSize: 15,
  },
  textSmall: {
    fontSize: 12,
  },
});
