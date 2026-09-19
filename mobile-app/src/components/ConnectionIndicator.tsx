import React from 'react';
import { StyleSheet, Text, View } from 'react-native';
import {
    colors,
    radius,
    spacing,
    typography,
} from '../constants/theme';

export type ConnectionStatus =
  | 'connected'
  | 'connecting'
  | 'disconnected';

const LABELS: Record<ConnectionStatus, string> = {
  connected: '연결됨',
  connecting: '연결 중',
  disconnected: '연결 끊김',
};

function getConnectionColor(status: ConnectionStatus) {
  switch (status) {
    case 'connected':
      return colors.connConnected;

    case 'connecting':
      return colors.connConnecting;

    case 'disconnected':
      return colors.connDisconnected;
  }
}

export function ConnectionIndicator({
  status,
}: {
  status: ConnectionStatus;
}) {
  const color = getConnectionColor(status);

  return (
    <View style={styles.container}>
      <View
        style={[
          styles.dot,
          {
            backgroundColor: color,
          },
        ]}
      />

      <Text
        style={[
          styles.label,
          {
            color,
          },
        ]}
      >
        {LABELS[status]}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flexDirection: 'row',
    alignItems: 'center',

    backgroundColor: colors.surface,

    borderRadius: radius.pill,

    paddingVertical: spacing.xs,
    paddingHorizontal: spacing.md,

    borderWidth: 1,
    borderColor: colors.border,
  },

  dot: {
    width: 8,
    height: 8,
    borderRadius: 4,
    marginRight: spacing.sm,
  },

  label: {
    ...typography.caption,
  },
});