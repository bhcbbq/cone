export const colors = {
  background: '#0B1220',
  surface: '#121B2E',
  surfaceElevated: '#182338',
  border: '#243146',

  textPrimary: '#F2F5FA',
  textSecondary: '#8C9BB5',
  textMuted: '#5C6B85',

  accent: '#FF7A29',
  accentSoft: '#FF7A2922',

  statusIdle: '#8C9BB5',
  statusDriving: '#3DDC97',
  statusStopped: '#FF5470',
  statusReturning: '#4FA8FF',
  statusArrived: '#FFC53D',

  connConnected: '#3DDC97',
  connConnecting: '#FFC53D',
  connDisconnected: '#FF5470',

  danger: '#FF5470',
  warning: '#FFC53D',
  success: '#3DDC97',
  info: '#4FA8FF',
} as const;

export const spacing = {
  xs: 4,
  sm: 8,
  md: 16,
  lg: 24,
  xl: 32,
} as const;

export const radius = {
  sm: 8,
  md: 14,
  lg: 20,
  pill: 999,
} as const;

export const typography = {
  hudNumber: {
    fontSize: 44,
    fontWeight: '800' as const,
    letterSpacing: -0.5,
  },

  h1: {
    fontSize: 22,
    fontWeight: '700' as const,
  },

  h2: {
    fontSize: 17,
    fontWeight: '700' as const,
  },

  body: {
    fontSize: 14,
    fontWeight: '400' as const,
  },

  caption: {
    fontSize: 12,
    fontWeight: '500' as const,
    letterSpacing: 0.4,
  },

  eyebrow: {
    fontSize: 11,
    fontWeight: '700' as const,
    letterSpacing: 1.2,
  },
};