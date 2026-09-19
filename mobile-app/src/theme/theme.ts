/**
 * 디자인 토큰 — 밝고 깔끔한 "제품용" 톤
 * 콘셉트: 현장에서 장갑 낀 손으로도 빠르게 조작 가능한 산업용 제어 UI.
 *   - 배경은 차가운 화이트 계열(#F5F7FA)로 눈부심 없이 밝게.
 *   - 라바콘을 연상시키는 시그니처 오렌지를 포인트 컬러로 사용하되,
 *     넓은 면적이 아닌 버튼/강조 요소에만 절제해서 사용.
 *   - 상태 색상(주행/정지/도착/복귀)은 채도 높은 컬러로 한눈에 구분되게.
 */

export const colors = {
  // Base
  background: '#F5F7FA',
  surface: '#FFFFFF',
  surfaceMuted: '#EEF1F6',
  border: '#E2E7EF',

  // Text
  textPrimary: '#16202E',
  textSecondary: '#5B6B82',
  textMuted: '#93A0B4',

  // Signature accent - 라바콘 오렌지
  accent: '#FF6A2B',
  accentSoft: '#FFEDE4',
  accentBorder: '#FFC9A8',

  // Status colors
  statusIdle: '#8C98AC',
  statusIdleSoft: '#EEF1F6',
  statusDriving: '#1DB876',
  statusDrivingSoft: '#E4F8EF',
  statusStopped: '#E23744',
  statusStoppedSoft: '#FDEAEC',
  statusReturning: '#2F7BF6',
  statusReturningSoft: '#E9F1FF',
  statusArrived: '#F5A623',
  statusArrivedSoft: '#FFF4E0',

  connConnected: '#1DB876',
  connConnecting: '#F5A623',
  connDisconnected: '#E23744',

  danger: '#E23744',
  dangerSoft: '#FDEAEC',
  warning: '#F5A623',
  warningSoft: '#FFF4E0',
  success: '#1DB876',
  info: '#2F7BF6',

  white: '#FFFFFF',
} as const;

export const spacing = {
  xs: 4,
  sm: 8,
  md: 16,
  lg: 24,
  xl: 32,
} as const;

export const radius = {
  sm: 10,
  md: 16,
  lg: 22,
  pill: 999,
} as const;

export const shadow = {
  card: {
    shadowColor: '#0F1B2D',
    shadowOffset: { width: 0, height: 6 },
    shadowOpacity: 0.06,
    shadowRadius: 16,
    elevation: 3,
  },
  button: {
    shadowColor: '#0F1B2D',
    shadowOffset: { width: 0, height: 3 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 2,
  },
} as const;

export const typography = {
  hudNumber: {
    fontSize: 42,
    fontWeight: '800' as const,
    letterSpacing: -0.5,
  },
  h1: {
    fontSize: 21,
    fontWeight: '800' as const,
  },
  h2: {
    fontSize: 16,
    fontWeight: '700' as const,
  },
  body: {
    fontSize: 14,
    fontWeight: '500' as const,
  },
  caption: {
    fontSize: 12,
    fontWeight: '600' as const,
    letterSpacing: 0.2,
  },
  eyebrow: {
    fontSize: 11,
    fontWeight: '800' as const,
    letterSpacing: 1.2,
  },
};

export function statusColor(status: string): string {
  switch (status) {
    case 'idle': return colors.statusIdle;
    case 'driving': return colors.statusDriving;
    case 'stopped': return colors.statusStopped;
    case 'arrived': return colors.statusArrived;
    case 'returning': return colors.statusReturning;
    case 'returnCompleted': return colors.statusDriving;
    default: return colors.textSecondary;
  }
}

export function statusSoftColor(status: string): string {
  switch (status) {
    case 'idle': return colors.statusIdleSoft;
    case 'driving': return colors.statusDrivingSoft;
    case 'stopped': return colors.statusStoppedSoft;
    case 'arrived': return colors.statusArrivedSoft;
    case 'returning': return colors.statusReturningSoft;
    case 'returnCompleted': return colors.statusDrivingSoft;
    default: return colors.surfaceMuted;
  }
}

export function connectionColor(status: string): string {
  switch (status) {
    case 'connected': return colors.connConnected;
    case 'connecting': return colors.connConnecting;
    case 'disconnected': return colors.connDisconnected;
    default: return colors.textSecondary;
  }
}
