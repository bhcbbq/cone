import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { RobotStatus } from '../types/robotUi';
import { colors, typography, spacing, radius, shadow } from '../theme/theme';

interface Props {
  robotStatus: RobotStatus;
  targetDistance: number;
}

/**
 * robotStatus 가 'arrived' 또는 'returnCompleted' 일 때만 렌더링되는
 * 강조 배너. 목표 지점 도착/복귀 완료를 스크롤 없이도 바로 인지할 수 있도록
 * 화면 상단부에 배치하는 것을 권장합니다.
 */
export function ArrivalBanner({ robotStatus, targetDistance }: Props) {
  if (robotStatus !== 'arrived' && robotStatus !== 'returnCompleted') return null;

  const isReturn = robotStatus === 'returnCompleted';

  return (
    <View style={[styles.card, shadow.card, isReturn ? styles.returnCard : styles.arrivedCard]}>
      <View style={[styles.iconCircle, { backgroundColor: isReturn ? colors.statusDriving : colors.statusArrived }]}>
        <Ionicons name="checkmark" size={20} color={colors.white} />
      </View>
      <View style={styles.textWrap}>
        <Text style={styles.title}>{isReturn ? '출발 지점 복귀 완료' : '목표 지점 도착 완료'}</Text>
        <Text style={styles.desc}>
          {isReturn
            ? '로봇이 출발 위치로 복귀를 마쳤습니다.'
            : `목표 거리 ${targetDistance.toFixed(1)}m 지점에 도착했습니다.`}
        </Text>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  card: {
    flexDirection: 'row',
    alignItems: 'center',
    borderRadius: radius.lg,
    borderWidth: 1.5,
    padding: spacing.md,
    marginBottom: spacing.md,
  },
  arrivedCard: {
    backgroundColor: colors.statusArrivedSoft,
    borderColor: colors.statusArrived + '55',
  },
  returnCard: {
    backgroundColor: colors.statusDrivingSoft,
    borderColor: colors.statusDriving + '55',
  },
  iconCircle: {
    width: 38,
    height: 38,
    borderRadius: 19,
    alignItems: 'center',
    justifyContent: 'center',
    marginRight: spacing.sm,
  },
  textWrap: {
    flex: 1,
  },
  title: {
    ...typography.h2,
    fontSize: 15,
    color: colors.textPrimary,
  },
  desc: {
    ...typography.caption,
    fontWeight: '500',
    color: colors.textSecondary,
    marginTop: 2,
  },
});
