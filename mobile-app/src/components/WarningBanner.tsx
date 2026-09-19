import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { WarningFlags } from '../types/robotUi';
import { colors, typography, spacing, radius } from '../theme/theme';

interface WarningDef {
  key: keyof WarningFlags;
  label: string;
  description: string;
  icon: keyof typeof Ionicons.glyphMap;
}

/**
 * 표시할 경고 항목 정의.
 * 새 센서(예: 배터리 부족)가 추가되면 WarningFlags 타입과 이 배열에
 * 한 줄씩만 추가하면 됩니다. 화면 컴포넌트는 수정할 필요 없습니다.
 */
const WARNING_DEFS: WarningDef[] = [
  { key: 'tiltDetected', label: '전도 감지', description: '로봇이 기울어지거나 넘어진 것으로 감지되었습니다.', icon: 'warning' },
  { key: 'obstacleDetected', label: '장애물 감지', description: '주행 경로에서 장애물이 감지되었습니다.', icon: 'alert-circle' },
  { key: 'laneDetectionFailed', label: '차선 인식 실패', description: '카메라가 차선을 인식하지 못하고 있습니다.', icon: 'eye-off' },
  { key: 'communicationLost', label: '통신 끊김', description: '로봇과의 통신이 끊어졌습니다.', icon: 'cloud-offline' },
];

export function WarningBanner({ warnings }: { warnings?: WarningFlags }) {
  if (!warnings) return null;
  const active = WARNING_DEFS.filter((w) => warnings[w.key]);
  if (active.length === 0) return null;

  return (
    <View style={styles.container}>
      {active.map((item) => (
        <View key={item.key} style={styles.row}>
          <View style={styles.iconCircle}>
            <Ionicons name={item.icon} size={16} color={colors.danger} />
          </View>
          <View style={styles.textWrap}>
            <Text style={styles.title}>{item.label}</Text>
            <Text style={styles.desc}>{item.description}</Text>
          </View>
        </View>
      ))}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    backgroundColor: colors.dangerSoft,
    borderWidth: 1,
    borderColor: colors.danger + '3D',
    borderRadius: radius.md,
    padding: spacing.sm,
    marginBottom: spacing.md,
  },
  row: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    paddingVertical: 6,
    paddingHorizontal: 4,
  },
  iconCircle: {
    width: 26,
    height: 26,
    borderRadius: 13,
    backgroundColor: colors.white,
    alignItems: 'center',
    justifyContent: 'center',
    marginRight: spacing.sm,
  },
  textWrap: {
    flex: 1,
  },
  title: {
    ...typography.h2,
    fontSize: 13,
    color: colors.danger,
  },
  desc: {
    ...typography.caption,
    fontWeight: '500',
    color: colors.textSecondary,
    marginTop: 1,
  },
});
