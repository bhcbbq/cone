import React from 'react';

import {
  Pressable,
  StyleSheet,
  Text,
  View,
} from 'react-native';

import { Ionicons } from '@expo/vector-icons';

import { RobotStatus } from '../types/robotUi';

import {
  colors,
  radius,
  shadow,
  spacing,
  typography,
} from '../theme/theme';

interface Props {
  robotStatus: RobotStatus;

  targetDistance: number;

  isBleConnected: boolean;

  onStart: () => void;

  onStop: () => void;

  onReturn?: () => void;

  onResetDistance?: () => void;
}

export function ControlButtons({
  robotStatus,
  targetDistance,
  isBleConnected,

  onStart,
  onStop,

  onReturn,
  onResetDistance,
}: Props) {
  const isDriving =
    robotStatus === 'driving';

  const isReturning =
    robotStatus ===
    'returning';

  const isBusy =
    isDriving || isReturning;

  // START 가능 상태
  const canStart =
    isBleConnected &&
    targetDistance > 0 &&
    (
      robotStatus === 'idle' ||
      robotStatus === 'stopped' ||
      robotStatus === 'arrived' ||
      robotStatus ===
        'returnCompleted'
    );

  // 주행 중에는 같은 버튼이 STOP
  const mainButtonEnabled =
    isDriving || canStart;

  // RESET은 움직이는 중에는 비활성
  const canReset =
    !isBusy;

  // RETURN은 아직 실제 기능 안 붙었으면
  // "준비 중" 상태로 표시
  const canReturn =
    !!onReturn &&
    robotStatus === 'arrived';

  const handleMainPress = () => {
    if (isDriving) {
      onStop();
      return;
    }

    if (canStart) {
      onStart();
    }
  };

  return (
    <View>
      {/* ========================
          메인 START / STOP
      ======================== */}

      <Pressable
        onPress={
          handleMainPress
        }
        disabled={
          !mainButtonEnabled
        }
        style={({ pressed }) => [
          styles.mainButton,

          isDriving
            ? styles.stopButton
            : styles.startButton,

          !mainButtonEnabled &&
            styles.mainButtonDisabled,

          pressed &&
            mainButtonEnabled && {
              opacity: 0.9,
            },
        ]}
      >
        <Ionicons
          name={
            isDriving
              ? 'stop'
              : 'play'
          }
          size={23}
          color={
            mainButtonEnabled
              ? colors.white
              : colors.textMuted
          }
        />

        <Text
          style={[
            styles.mainButtonText,
            {
              color:
                mainButtonEnabled
                  ? colors.white
                  : colors.textMuted,
            },
          ]}
        >
          {isDriving
            ? 'STOP'
            : 'START'}
        </Text>
      </Pressable>

      {/* ========================
          RESET / RETURN
      ======================== */}

      <View
        style={
          styles.secondaryRow
        }
      >
        <Pressable
          onPress={
            onResetDistance
          }
          disabled={
            !onResetDistance ||
            !canReset
          }
          style={({
            pressed,
          }) => [
            styles.secondaryButton,

            (!onResetDistance ||
              !canReset) &&
              styles.secondaryButtonDisabled,

            pressed &&
              onResetDistance &&
              canReset && {
                opacity: 0.82,
              },
          ]}
        >
          <Ionicons
            name="refresh"
            size={18}
            color={
              canReset
                ? colors.textSecondary
                : colors.textMuted
            }
          />

          <Text
            style={[
              styles.secondaryText,
              {
                color:
                  canReset
                    ? colors.textPrimary
                    : colors.textMuted,
              },
            ]}
          >
            RESET
          </Text>
        </Pressable>

        <Pressable
          onPress={onReturn}
          disabled={!canReturn}
          style={({ pressed }) => [
            styles.secondaryButton,
            styles.returnButton,

            !canReturn &&
              styles.secondaryButtonDisabled,

            pressed &&
              canReturn && {
                opacity: 0.82,
              },
          ]}
        >
          <Ionicons
            name="return-down-back"
            size={18}
            color={
              canReturn
                ? colors.info
                : colors.textMuted
            }
          />

          <View
            style={{
              alignItems:
                'center',
            }}
          >
            <Text
              style={[
                styles.secondaryText,
                {
                  color:
                    canReturn
                      ? colors.info
                      : colors.textMuted,
                },
              ]}
            >
              RETURN
            </Text>

            {!onReturn && (
              <Text
                style={
                  styles.preparingText
                }
              >
                (준비 중)
              </Text>
            )}
          </View>
        </Pressable>
      </View>

      {!isBleConnected && (
        <Text
          style={styles.hint}
        >
          ESP32와 연결되어야
          주행을 시작할 수 있습니다.
        </Text>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  mainButton: {
    width: '100%',

    flexDirection: 'row',

    alignItems: 'center',

    justifyContent:
      'center',

    borderRadius:
      radius.md,

    paddingVertical: 18,

    ...shadow.button,
  },

  startButton: {
    backgroundColor:
      colors.statusDriving,
  },

  stopButton: {
    backgroundColor:
      colors.statusStopped,
  },

  mainButtonDisabled: {
    backgroundColor:
      colors.surfaceMuted,

    shadowOpacity: 0,

    elevation: 0,
  },

  mainButtonText: {
    ...typography.h1,

    fontSize: 18,

    letterSpacing: 0.5,

    marginLeft:
      spacing.sm,
  },

  secondaryRow: {
    flexDirection: 'row',

    marginTop:
      spacing.md,
  },

  secondaryButton: {
    flex: 1,

    minHeight: 58,

    flexDirection: 'row',

    alignItems: 'center',

    justifyContent:
      'center',

    backgroundColor:
      colors.surface,

    borderWidth: 1.5,

    borderColor:
      colors.border,

    borderRadius:
      radius.md,

    paddingHorizontal:
      spacing.sm,

    ...shadow.button,
  },

  returnButton: {
    marginLeft:
      spacing.sm,
  },

  secondaryButtonDisabled: {
    opacity: 0.48,

    shadowOpacity: 0,

    elevation: 0,
  },

  secondaryText: {
    ...typography.h2,

    fontSize: 13,

    marginLeft: 7,
  },

  preparingText: {
    ...typography.caption,

    fontSize: 10,

    color:
      colors.textMuted,

    marginLeft: 7,

    marginTop: 1,
  },

  hint: {
    ...typography.caption,

    color:
      colors.textMuted,

    textAlign: 'center',

    marginTop:
      spacing.md,
  },
});