import React, { useEffect, useState } from 'react';
import { View, Text, TextInput, Pressable, StyleSheet } from 'react-native';
import { colors, typography, spacing, radius, shadow } from '../theme/theme';

const PRESETS = [50, 100, 200];

interface Props {
  value: number;
  editable: boolean;
  onChange: (meters: number) => void;
}

export function DistanceInputCard({ value, editable, onChange }: Props) {
  const [text, setText] = useState(value > 0 ? String(value) : '');

  useEffect(() => {
    setText(value > 0 ? String(value) : '');
  }, [value]);

  const commit = (raw: string) => {
    const parsed = Number(raw.replace(/[^0-9.]/g, ''));
    onChange(Number.isFinite(parsed) ? parsed : 0);
  };

  return (
    <View style={[styles.card, shadow.card]}>
      <Text style={styles.label}>목표 거리</Text>

      <View style={[styles.inputRow, !editable && styles.inputRowDisabled]}>
        <TextInput
          style={styles.input}
          keyboardType="decimal-pad"
          placeholder="0"
          placeholderTextColor={colors.textMuted}
          value={text}
          editable={editable}
          onChangeText={setText}
          onEndEditing={() => commit(text)}
          onBlur={() => commit(text)}
        />
        <Text style={styles.unit}>m</Text>
      </View>

      <View style={styles.presetRow}>
        {PRESETS.map((preset) => {
          const selected = value === preset;
          return (
            <Pressable
              key={preset}
              disabled={!editable}
              onPress={() => {
                setText(String(preset));
                onChange(preset);
              }}
              style={({ pressed }) => [
                styles.presetChip,
                selected && styles.presetChipSelected,
                !editable && styles.presetChipDisabled,
                pressed && editable && { opacity: 0.85 },
              ]}
            >
              <Text style={[styles.presetText, selected && styles.presetTextSelected]}>{preset}m</Text>
            </Pressable>
          );
        })}
      </View>

      {!editable && <Text style={styles.hint}>주행/복귀 중에는 목표 거리를 변경할 수 없습니다.</Text>}
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
  label: {
    ...typography.eyebrow,
    color: colors.textMuted,
    marginBottom: spacing.sm,
  },
  inputRow: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: colors.surfaceMuted,
    borderRadius: radius.md,
    borderWidth: 1,
    borderColor: colors.border,
    paddingHorizontal: spacing.md,
  },
  inputRowDisabled: {
    opacity: 0.55,
  },
  input: {
    flex: 1,
    ...typography.hudNumber,
    fontSize: 30,
    color: colors.textPrimary,
    paddingVertical: 14,
  },
  unit: {
    ...typography.h2,
    color: colors.textSecondary,
    marginLeft: spacing.sm,
  },
  presetRow: {
    flexDirection: 'row',
    marginTop: spacing.sm,
  },
  presetChip: {
    borderWidth: 1.5,
    borderColor: colors.border,
    backgroundColor: colors.surface,
    borderRadius: radius.pill,
    paddingVertical: 7,
    paddingHorizontal: spacing.md,
    marginRight: spacing.sm,
  },
  presetChipSelected: {
    backgroundColor: colors.accentSoft,
    borderColor: colors.accent,
  },
  presetChipDisabled: {
    opacity: 0.5,
  },
  presetText: {
    ...typography.caption,
    color: colors.textSecondary,
  },
  presetTextSelected: {
    color: colors.accent,
  },
  hint: {
    ...typography.caption,
    fontWeight: '500',
    color: colors.textMuted,
    marginTop: spacing.sm,
  },
});
