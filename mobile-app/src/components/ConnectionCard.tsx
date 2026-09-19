import React from 'react';
import { View, Text, Pressable, StyleSheet, ActivityIndicator } from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { ConnectionStatus } from '../types/robotUi';
import { colors, typography, spacing, radius, shadow, connectionColor } from '../theme/theme';

const LABELS: Record<ConnectionStatus, string> = {
  connected: '연결됨',
  connecting: '연결 중',
  disconnected: '연결 끊김',
};

interface Props {
  status: ConnectionStatus;
  deviceName?: string;
  onConnect: () => void;
  onDisconnect: () => void;
}

export function ConnectionCard({ status, deviceName, onConnect, onDisconnect }: Props) {
  const color = connectionColor(status);
  const isConnected = status === 'connected';
  const isConnecting = status === 'connecting';

  return (
    <View style={[styles.card, shadow.card]}>
      <View style={styles.left}>
        <View style={[styles.iconCircle, { backgroundColor: color + '1F' }]}>
          <Ionicons
            name={isConnected ? 'bluetooth' : 'bluetooth-outline'}
            size={20}
            color={color}
          />
        </View>
        <View style={styles.textCol}>
          <Text style={styles.deviceLabel}>{deviceName ?? 'ESP32 라바콘 로봇'}</Text>
          <View style={styles.statusRow}>
            {isConnecting && <ActivityIndicator size="small" color={color} style={{ marginRight: 6 }} />}
            {!isConnecting && <View style={[styles.dot, { backgroundColor: color }]} />}
            <Text style={[styles.statusText, { color }]}>{LABELS[status]}</Text>
          </View>
        </View>
      </View>

      <Pressable
        onPress={isConnected ? onDisconnect : onConnect}
        disabled={isConnecting}
        style={({ pressed }) => [
          styles.actionButton,
          isConnected ? styles.disconnectButton : styles.connectButton,
          isConnecting && styles.actionButtonDisabled,
          pressed && !isConnecting && { opacity: 0.85 },
        ]}
      >
        <Text style={[styles.actionText, isConnected ? styles.disconnectText : styles.connectText]}>
          {isConnecting ? '연결 중…' : isConnected ? '연결 해제' : '연결하기'}
        </Text>
      </Pressable>
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
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  left: {
    flexDirection: 'row',
    alignItems: 'center',
    flexShrink: 1,
  },
  iconCircle: {
    width: 40,
    height: 40,
    borderRadius: 20,
    alignItems: 'center',
    justifyContent: 'center',
    marginRight: spacing.sm,
  },
  textCol: {
    flexShrink: 1,
  },
  deviceLabel: {
    ...typography.h2,
    fontSize: 14,
    color: colors.textPrimary,
  },
  statusRow: {
    flexDirection: 'row',
    alignItems: 'center',
    marginTop: 3,
  },
  dot: {
    width: 7,
    height: 7,
    borderRadius: 4,
    marginRight: 6,
  },
  statusText: {
    ...typography.caption,
  },
  actionButton: {
    paddingVertical: 9,
    paddingHorizontal: spacing.md,
    borderRadius: radius.pill,
    borderWidth: 1.5,
    marginLeft: spacing.sm,
  },
  connectButton: {
    backgroundColor: colors.accent,
    borderColor: colors.accent,
  },
  disconnectButton: {
    backgroundColor: colors.surface,
    borderColor: colors.border,
  },
  actionButtonDisabled: {
    opacity: 0.6,
  },
  actionText: {
    ...typography.caption,
    fontSize: 13,
  },
  connectText: {
    color: colors.white,
  },
  disconnectText: {
    color: colors.textSecondary,
  },
});
