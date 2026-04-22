/**
 * JK BMS Power Core Card
 * A high-tech battery monitoring card for Home Assistant
 * 
 * Installation:
 * 1. Copy this file to /config/www/jk-bms-card.js
 * 2. Add to Lovelace resources:
 *    URL: /local/jk-bms-card.js
 *    Type: JavaScript Module
 * 3. Add card via UI or YAML
 */

const CARD_VERSION = '1.0.0';

// Log card info
console.info(
  `%c JK-BMS-CARD %c v${CARD_VERSION} `,
  'background: #00f0ff; color: #0a0e14; font-weight: bold;',
  'background: #0a0e14; color: #00f0ff;'
);

// ============================================
// STYLES
// ============================================
const cardStyles = `
  :host {
    --bms-bg-primary: #0a0e14;
    --bms-bg-secondary: #111820;
    --bms-bg-tertiary: #1a2332;
    --bms-accent-cyan: #00f0ff;
    --bms-accent-green: #00ff88;
    --bms-accent-amber: #ffaa00;
    --bms-accent-red: #ff3366;
    --bms-accent-purple: #aa44ff;
    --bms-text-primary: #e8f4f8;
    --bms-text-secondary: #7a8a9a;
    --bms-text-dim: #4a5a6a;
  }

  * {
    box-sizing: border-box;
  }

  .card-container {
    background: var(--bms-bg-primary);
    border-radius: var(--ha-card-border-radius, 12px);
    padding: 16px;
    font-family: 'Segoe UI', Roboto, sans-serif;
    color: var(--bms-text-primary);
    position: relative;
    overflow: hidden;
  }

  .card-container::before {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background-image: 
      linear-gradient(rgba(0, 240, 255, 0.02) 1px, transparent 1px),
      linear-gradient(90deg, rgba(0, 240, 255, 0.02) 1px, transparent 1px);
    background-size: 30px 30px;
    pointer-events: none;
  }

  /* Header */
  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 16px;
    padding-bottom: 12px;
    border-bottom: 1px solid rgba(0, 240, 255, 0.2);
    position: relative;
  }

  .title {
    font-size: 1.1rem;
    font-weight: 600;
    letter-spacing: 1px;
    text-transform: uppercase;
    color: var(--bms-accent-cyan);
    margin: 0;
  }

  .status {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.75rem;
    color: var(--bms-text-secondary);
  }

  .status-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--bms-accent-green);
    box-shadow: 0 0 8px rgba(0, 255, 136, 0.6);
    animation: pulse 2s ease-in-out infinite;
  }

  .status-dot.offline {
    background: var(--bms-accent-red);
    box-shadow: 0 0 8px rgba(255, 51, 102, 0.6);
    animation: none;
  }

  @keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.5; }
  }

  /* Main Layout */
  .main-content {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
  }

  /* SOC Gauge */
  .soc-section {
    grid-column: 1 / -1;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 30px;
    padding: 16px;
    background: var(--bms-bg-secondary);
    border-radius: 10px;
    border: 1px solid rgba(0, 240, 255, 0.1);
  }

  .soc-gauge {
    position: relative;
    width: 140px;
    height: 140px;
  }

  .soc-gauge svg {
    transform: rotate(-90deg);
    width: 100%;
    height: 100%;
  }

  .soc-bg {
    fill: none;
    stroke: var(--bms-bg-tertiary);
    stroke-width: 10;
  }

  .soc-progress {
    fill: none;
    stroke: url(#socGrad);
    stroke-width: 10;
    stroke-linecap: round;
    stroke-dasharray: 377;
    stroke-dashoffset: 377;
    transition: stroke-dashoffset 1s ease-out;
    filter: drop-shadow(0 0 6px rgba(0, 240, 255, 0.5));
  }

  .soc-center {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    text-align: center;
  }

  .soc-value {
    font-size: 2.2rem;
    font-weight: 700;
    color: var(--bms-accent-cyan);
    line-height: 1;
  }

  .soc-label {
    font-size: 0.65rem;
    color: var(--bms-text-dim);
    text-transform: uppercase;
    letter-spacing: 1px;
  }

  .soc-stats {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .stat-row {
    display: flex;
    align-items: center;
    gap: 10px;
  }

  .stat-icon {
    width: 32px;
    height: 32px;
    display: flex;
    align-items: center;
    justify-content: center;
    background: var(--bms-bg-tertiary);
    border-radius: 6px;
    font-size: 1rem;
  }

  .stat-info {
    flex: 1;
  }

  .stat-value {
    font-size: 1.2rem;
    font-weight: 600;
    color: var(--bms-text-primary);
  }

  .stat-value.charging {
    color: var(--bms-accent-green);
  }

  .stat-value.discharging {
    color: var(--bms-accent-amber);
  }

  .stat-name {
    font-size: 0.65rem;
    color: var(--bms-text-dim);
    text-transform: uppercase;
  }

  /* Info Panels */
  .info-panel {
    background: var(--bms-bg-secondary);
    border-radius: 8px;
    padding: 12px;
    border: 1px solid rgba(0, 240, 255, 0.1);
  }

  .panel-title {
    font-size: 0.65rem;
    color: var(--bms-text-dim);
    text-transform: uppercase;
    letter-spacing: 1px;
    margin-bottom: 8px;
    display: flex;
    align-items: center;
    gap: 6px;
  }

  .panel-title::before {
    content: '◆';
    color: var(--bms-accent-cyan);
    font-size: 0.5rem;
  }

  .panel-value {
    font-size: 1.6rem;
    font-weight: 700;
    color: var(--bms-text-primary);
  }

  .panel-value .unit {
    font-size: 0.9rem;
    font-weight: 400;
    color: var(--bms-text-secondary);
    margin-left: 4px;
  }

  .panel-bar {
    height: 4px;
    background: var(--bms-bg-tertiary);
    border-radius: 2px;
    margin-top: 8px;
    overflow: hidden;
  }

  .panel-bar-fill {
    height: 100%;
    border-radius: 2px;
    transition: width 0.5s ease-out;
  }

  .panel-bar-fill.voltage {
    background: linear-gradient(90deg, var(--bms-accent-red), var(--bms-accent-amber), var(--bms-accent-green));
  }

  .panel-bar-fill.soh {
    background: var(--bms-accent-green);
  }

  /* Cell Grid */
  .cells-section {
    grid-column: 1 / -1;
    background: var(--bms-bg-secondary);
    border-radius: 8px;
    padding: 12px;
    border: 1px solid rgba(0, 240, 255, 0.1);
  }

  .cells-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 10px;
  }

  .cells-stats {
    display: flex;
    gap: 12px;
  }

  .cell-stat {
    text-align: center;
  }

  .cell-stat-value {
    font-size: 0.85rem;
    font-weight: 600;
  }

  .cell-stat-value.delta { color: var(--bms-accent-cyan); }
  .cell-stat-value.max { color: var(--bms-accent-green); }
  .cell-stat-value.min { color: var(--bms-accent-amber); }

  .cell-stat-label {
    font-size: 0.55rem;
    color: var(--bms-text-dim);
    text-transform: uppercase;
  }

  .cells-grid {
    display: grid;
    grid-template-columns: repeat(8, 1fr);
    gap: 6px;
  }

  .cell {
    background: var(--bms-bg-tertiary);
    border-radius: 4px;
    padding: 6px 4px;
    text-align: center;
    border: 1px solid transparent;
    transition: all 0.2s ease;
  }

  .cell:hover {
    border-color: var(--bms-accent-cyan);
    transform: translateY(-1px);
  }

  .cell-num {
    font-size: 0.5rem;
    color: var(--bms-text-dim);
    margin-bottom: 2px;
  }

  .cell-voltage {
    font-size: 0.75rem;
    font-weight: 600;
    color: var(--bms-text-primary);
  }

  .cell.high .cell-voltage { color: var(--bms-accent-green); }
  .cell.low .cell-voltage { color: var(--bms-accent-amber); }

  .cell-bar {
    height: 2px;
    background: var(--bms-bg-primary);
    border-radius: 1px;
    margin-top: 4px;
    overflow: hidden;
  }

  .cell-bar-fill {
    height: 100%;
    background: var(--bms-accent-cyan);
    transition: width 0.3s ease;
  }

  .cell.high .cell-bar-fill { background: var(--bms-accent-green); }
  .cell.low .cell-bar-fill { background: var(--bms-accent-amber); }

  /* Temperature Section */
  .temp-section {
    display: flex;
    gap: 12px;
  }

  .temp-item {
    flex: 1;
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 8px;
    background: var(--bms-bg-tertiary);
    border-radius: 6px;
  }

  .temp-icon {
    font-size: 1.2rem;
  }

  .temp-value {
    font-size: 1rem;
    font-weight: 600;
    color: var(--bms-text-primary);
  }

  .temp-label {
    font-size: 0.55rem;
    color: var(--bms-text-dim);
    text-transform: uppercase;
  }

  /* Energy Stats */
  .energy-section {
    grid-column: 1 / -1;
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 8px;
  }

  .energy-item {
    background: var(--bms-bg-secondary);
    border-radius: 6px;
    padding: 10px;
    text-align: center;
    border: 1px solid rgba(0, 240, 255, 0.1);
  }

  .energy-icon {
    font-size: 1rem;
    margin-bottom: 4px;
  }

  .energy-value {
    font-size: 0.95rem;
    font-weight: 600;
    color: var(--bms-text-primary);
  }

  .energy-label {
    font-size: 0.55rem;
    color: var(--bms-text-dim);
    text-transform: uppercase;
    margin-top: 2px;
  }

  /* Balance Indicator */
  .balance-indicator {
    grid-column: 1 / -1;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    padding: 8px;
    background: var(--bms-bg-secondary);
    border-radius: 6px;
    border: 1px solid rgba(0, 240, 255, 0.1);
  }

  .balance-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--bms-text-dim);
  }

  .balance-dot.active {
    background: var(--bms-accent-green);
    box-shadow: 0 0 8px rgba(0, 255, 136, 0.6);
    animation: pulse 1s ease-in-out infinite;
  }

  .balance-text {
    font-size: 0.75rem;
    color: var(--bms-text-secondary);
  }

  /* Compact Mode */
  .card-container.compact .main-content {
    grid-template-columns: 1fr;
  }

  .card-container.compact .soc-section {
    flex-direction: column;
    gap: 12px;
  }

  .card-container.compact .soc-gauge {
    width: 100px;
    height: 100px;
  }

  .card-container.compact .soc-value {
    font-size: 1.6rem;
  }

  .card-container.compact .cells-grid {
    grid-template-columns: repeat(4, 1fr);
  }

  .card-container.compact .energy-section {
    grid-template-columns: repeat(2, 1fr);
  }

  /* Unavailable state */
  .unavailable {
    opacity: 0.5;
  }

  .unavailable .soc-progress {
    stroke: var(--bms-text-dim);
    filter: none;
  }
`;

// ============================================
// CARD EDITOR
// ============================================
class JKBMSCardEditor extends HTMLElement {
  constructor() {
    super();
    this._config = {};
  }

  setConfig(config) {
    this._config = config;
    this.render();
  }

  get _title() {
    return this._config.title || 'JK BMS Power Core';
  }

  get _compact() {
    return this._config.compact || false;
  }

  get _show_cells() {
    return this._config.show_cells !== false;
  }

  get _show_temps() {
    return this._config.show_temps !== false;
  }

  get _show_energy() {
    return this._config.show_energy !== false;
  }

  get _show_balance() {
    return this._config.show_balance !== false;
  }

  render() {
    if (!this.shadowRoot) {
      this.attachShadow({ mode: 'open' });
    }

    this.shadowRoot.innerHTML = `
      <style>
        .editor {
          padding: 16px;
        }
        .row {
          margin-bottom: 16px;
        }
        .row label {
          display: block;
          margin-bottom: 4px;
          font-weight: 500;
          color: var(--primary-text-color);
        }
        .row input[type="text"],
        .row select {
          width: 100%;
          padding: 8px;
          border: 1px solid var(--divider-color);
          border-radius: 4px;
          background: var(--card-background-color);
          color: var(--primary-text-color);
        }
        .row input[type="checkbox"] {
          margin-right: 8px;
        }
        .checkbox-row {
          display: flex;
          align-items: center;
          margin-bottom: 8px;
        }
        .section-title {
          font-weight: 600;
          margin: 16px 0 8px 0;
          padding-bottom: 4px;
          border-bottom: 1px solid var(--divider-color);
          color: var(--primary-text-color);
        }
        .entity-picker {
          margin-bottom: 8px;
        }
        .entity-picker label {
          font-size: 0.9em;
          color: var(--secondary-text-color);
        }
        ha-entity-picker {
          display: block;
          width: 100%;
        }
      </style>
      <div class="editor">
        <div class="row">
          <label>Card Title</label>
          <input type="text" id="title" value="${this._title}" />
        </div>

        <div class="section-title">Display Options</div>
        <div class="checkbox-row">
          <input type="checkbox" id="compact" ${this._compact ? 'checked' : ''} />
          <label for="compact">Compact Mode</label>
        </div>
        <div class="checkbox-row">
          <input type="checkbox" id="show_cells" ${this._show_cells ? 'checked' : ''} />
          <label for="show_cells">Show Cell Voltages</label>
        </div>
        <div class="checkbox-row">
          <input type="checkbox" id="show_temps" ${this._show_temps ? 'checked' : ''} />
          <label for="show_temps">Show Temperatures</label>
        </div>
        <div class="checkbox-row">
          <input type="checkbox" id="show_energy" ${this._show_energy ? 'checked' : ''} />
          <label for="show_energy">Show Energy Statistics</label>
        </div>
        <div class="checkbox-row">
          <input type="checkbox" id="show_balance" ${this._show_balance ? 'checked' : ''} />
          <label for="show_balance">Show Balance Indicator</label>
        </div>

        <div class="section-title">Main Sensors</div>
        <div class="entity-picker">
          <label>State of Charge (SOC)</label>
          <input type="text" id="soc_entity" value="${this._config.soc_entity || ''}" placeholder="sensor.jk_bms_soc" />
        </div>
        <div class="entity-picker">
          <label>State of Health (SOH)</label>
          <input type="text" id="soh_entity" value="${this._config.soh_entity || ''}" placeholder="sensor.jk_bms_soh" />
        </div>
        <div class="entity-picker">
          <label>Pack Voltage</label>
          <input type="text" id="voltage_entity" value="${this._config.voltage_entity || ''}" placeholder="sensor.jk_bms_voltage" />
        </div>
        <div class="entity-picker">
          <label>Current</label>
          <input type="text" id="current_entity" value="${this._config.current_entity || ''}" placeholder="sensor.jk_bms_current" />
        </div>
        <div class="entity-picker">
          <label>Power</label>
          <input type="text" id="power_entity" value="${this._config.power_entity || ''}" placeholder="sensor.jk_bms_power" />
        </div>

        <div class="section-title">Capacity Sensors</div>
        <div class="entity-picker">
          <label>Capacity Full</label>
          <input type="text" id="capacity_full_entity" value="${this._config.capacity_full_entity || ''}" placeholder="sensor.jk_bms_capacity_full" />
        </div>
        <div class="entity-picker">
          <label>Capacity Remaining</label>
          <input type="text" id="capacity_remaining_entity" value="${this._config.capacity_remaining_entity || ''}" placeholder="sensor.jk_bms_capacity_remaining" />
        </div>

        <div class="section-title">Cell Sensors</div>
        <div class="entity-picker">
          <label>Cell Delta</label>
          <input type="text" id="cell_delta_entity" value="${this._config.cell_delta_entity || ''}" placeholder="sensor.jk_bms_cell_delta" />
        </div>
        <div class="entity-picker">
          <label>Cell Max</label>
          <input type="text" id="cell_max_entity" value="${this._config.cell_max_entity || ''}" placeholder="sensor.jk_bms_cell_max" />
        </div>
        <div class="entity-picker">
          <label>Cell Min</label>
          <input type="text" id="cell_min_entity" value="${this._config.cell_min_entity || ''}" placeholder="sensor.jk_bms_cell_min" />
        </div>
        <div class="entity-picker">
          <label>Cell Entities Prefix (e.g., sensor.jk_bms_cell_)</label>
          <input type="text" id="cell_prefix" value="${this._config.cell_prefix || ''}" placeholder="sensor.jk_bms_cell_" />
        </div>
        <div class="entity-picker">
          <label>Number of Cells</label>
          <input type="text" id="cell_count" value="${this._config.cell_count || 16}" placeholder="16" />
        </div>

        <div class="section-title">Temperature Sensors</div>
        <div class="entity-picker">
          <label>Battery Temperature</label>
          <input type="text" id="battery_temp_entity" value="${this._config.battery_temp_entity || ''}" placeholder="sensor.jk_bms_battery_temp" />
        </div>
        <div class="entity-picker">
          <label>MOS Temperature</label>
          <input type="text" id="mos_temp_entity" value="${this._config.mos_temp_entity || ''}" placeholder="sensor.jk_bms_mos_temp" />
        </div>

        <div class="section-title">Energy Sensors</div>
        <div class="entity-picker">
          <label>Energy In</label>
          <input type="text" id="energy_in_entity" value="${this._config.energy_in_entity || ''}" placeholder="sensor.jk_bms_energy_in" />
        </div>
        <div class="entity-picker">
          <label>Energy Out</label>
          <input type="text" id="energy_out_entity" value="${this._config.energy_out_entity || ''}" placeholder="sensor.jk_bms_energy_out" />
        </div>
        <div class="entity-picker">
          <label>Cycle Count</label>
          <input type="text" id="cycle_count_entity" value="${this._config.cycle_count_entity || ''}" placeholder="sensor.jk_bms_cycle_count" />
        </div>
        <div class="entity-picker">
          <label>Cycle Capacity</label>
          <input type="text" id="cycle_capacity_entity" value="${this._config.cycle_capacity_entity || ''}" placeholder="sensor.jk_bms_cycle_capacity" />
        </div>

        <div class="section-title">Balance Sensor</div>
        <div class="entity-picker">
          <label>Balance Current</label>
          <input type="text" id="balance_current_entity" value="${this._config.balance_current_entity || ''}" placeholder="sensor.jk_bms_balance_current" />
        </div>
      </div>
    `;

    // Add event listeners
    const inputs = this.shadowRoot.querySelectorAll('input');
    inputs.forEach(input => {
      input.addEventListener('change', (e) => this._valueChanged(e));
      input.addEventListener('input', (e) => this._valueChanged(e));
    });
  }

  _valueChanged(ev) {
    if (!this._config) return;

    const target = ev.target;
    const configValue = target.id;

    let newValue;
    if (target.type === 'checkbox') {
      newValue = target.checked;
    } else if (configValue === 'cell_count') {
      newValue = parseInt(target.value) || 16;
    } else {
      newValue = target.value;
    }

    if (this._config[configValue] === newValue) return;

    const newConfig = {
      ...this._config,
      [configValue]: newValue,
    };

    const event = new CustomEvent('config-changed', {
      detail: { config: newConfig },
      bubbles: true,
      composed: true,
    });
    this.dispatchEvent(event);
  }
}

// ============================================
// MAIN CARD
// ============================================
class JKBMSCard extends HTMLElement {
  constructor() {
    super();
    this.attachShadow({ mode: 'open' });
    this._config = {};
    this._hass = null;
  }

  static getConfigElement() {
    return document.createElement('jk-bms-card-editor');
  }

  static getStubConfig() {
    return {
      title: 'JK BMS Power Core',
      compact: false,
      show_cells: true,
      show_temps: true,
      show_energy: true,
      show_balance: true,
      cell_count: 16,
      soc_entity: '',
      soh_entity: '',
      voltage_entity: '',
      current_entity: '',
      power_entity: '',
      capacity_full_entity: '',
      capacity_remaining_entity: '',
      cell_delta_entity: '',
      cell_max_entity: '',
      cell_min_entity: '',
      cell_prefix: '',
      battery_temp_entity: '',
      mos_temp_entity: '',
      energy_in_entity: '',
      energy_out_entity: '',
      cycle_count_entity: '',
      cycle_capacity_entity: '',
      balance_current_entity: '',
    };
  }

  setConfig(config) {
    this._config = {
      title: 'JK BMS Power Core',
      compact: false,
      show_cells: true,
      show_temps: true,
      show_energy: true,
      show_balance: true,
      cell_count: 16,
      ...config,
    };
    this.render();
  }

  set hass(hass) {
    this._hass = hass;
    this.updateValues();
  }

  getState(entityId) {
    if (!this._hass || !entityId) return null;
    const state = this._hass.states[entityId];
    if (!state || state.state === 'unavailable' || state.state === 'unknown') {
      return null;
    }
    return parseFloat(state.state);
  }

  getStateStr(entityId) {
    if (!this._hass || !entityId) return '--';
    const state = this._hass.states[entityId];
    if (!state || state.state === 'unavailable' || state.state === 'unknown') {
      return '--';
    }
    return state.state;
  }

  isAvailable(entityId) {
    if (!this._hass || !entityId) return false;
    const state = this._hass.states[entityId];
    return state && state.state !== 'unavailable' && state.state !== 'unknown';
  }

  render() {
    const config = this._config;
    const cellCount = config.cell_count || 16;

    this.shadowRoot.innerHTML = `
      <style>${cardStyles}</style>
      <ha-card>
        <div class="card-container ${config.compact ? 'compact' : ''}">
          <!-- Header -->
          <div class="header">
            <h2 class="title">${config.title}</h2>
            <div class="status">
              <span class="status-dot" id="statusDot"></span>
              <span id="statusText">ONLINE</span>
            </div>
          </div>

          <div class="main-content">
            <!-- SOC Section -->
            <div class="soc-section">
              <div class="soc-gauge">
                <svg viewBox="0 0 140 140">
                  <defs>
                    <linearGradient id="socGrad" x1="0%" y1="0%" x2="100%" y2="0%">
                      <stop offset="0%" style="stop-color:#00f0ff"/>
                      <stop offset="50%" style="stop-color:#00ff88"/>
                      <stop offset="100%" style="stop-color:#00f0ff"/>
                    </linearGradient>
                  </defs>
                  <circle class="soc-bg" cx="70" cy="70" r="60"/>
                  <circle class="soc-progress" cx="70" cy="70" r="60" id="socProgress"/>
                </svg>
                <div class="soc-center">
                  <div class="soc-value" id="socValue">--%</div>
                  <div class="soc-label">Charge</div>
                </div>
              </div>
              <div class="soc-stats">
                <div class="stat-row">
                  <div class="stat-icon">⚡</div>
                  <div class="stat-info">
                    <div class="stat-value" id="voltageValue">-- V</div>
                    <div class="stat-name">Pack Voltage</div>
                  </div>
                </div>
                <div class="stat-row">
                  <div class="stat-icon">🔌</div>
                  <div class="stat-info">
                    <div class="stat-value" id="currentValue">-- A</div>
                    <div class="stat-name">Current</div>
                  </div>
                </div>
                <div class="stat-row">
                  <div class="stat-icon">💡</div>
                  <div class="stat-info">
                    <div class="stat-value" id="powerValue">-- W</div>
                    <div class="stat-name">Power</div>
                  </div>
                </div>
                <div class="stat-row">
                  <div class="stat-icon">🔋</div>
                  <div class="stat-info">
                    <div class="stat-value" id="capacityValue">-- / -- Ah</div>
                    <div class="stat-name">Capacity</div>
                  </div>
                </div>
              </div>
            </div>

            <!-- Voltage Panel -->
            <div class="info-panel">
              <div class="panel-title">Pack Voltage</div>
              <div class="panel-value"><span id="voltagePanel">--</span><span class="unit">V</span></div>
              <div class="panel-bar">
                <div class="panel-bar-fill voltage" id="voltageBar" style="width: 0%"></div>
              </div>
            </div>

            <!-- SOH Panel -->
            <div class="info-panel">
              <div class="panel-title">State of Health</div>
              <div class="panel-value"><span id="sohValue">--</span><span class="unit">%</span></div>
              <div class="panel-bar">
                <div class="panel-bar-fill soh" id="sohBar" style="width: 0%"></div>
              </div>
            </div>

            ${config.show_temps ? `
            <!-- Temperature Section -->
            <div class="info-panel" style="grid-column: 1 / -1;">
              <div class="panel-title">Temperature</div>
              <div class="temp-section">
                <div class="temp-item">
                  <span class="temp-icon">🌡️</span>
                  <div>
                    <div class="temp-value" id="battTempValue">--°F</div>
                    <div class="temp-label">Battery</div>
                  </div>
                </div>
                <div class="temp-item">
                  <span class="temp-icon">🔥</span>
                  <div>
                    <div class="temp-value" id="mosTempValue">--°F</div>
                    <div class="temp-label">MOS FET</div>
                  </div>
                </div>
              </div>
            </div>
            ` : ''}

            ${config.show_cells ? `
            <!-- Cells Section -->
            <div class="cells-section">
              <div class="cells-header">
                <div class="panel-title">Cell Voltages (${cellCount}S)</div>
                <div class="cells-stats">
                  <div class="cell-stat">
                    <div class="cell-stat-value delta" id="cellDelta">-- mV</div>
                    <div class="cell-stat-label">Delta</div>
                  </div>
                  <div class="cell-stat">
                    <div class="cell-stat-value max" id="cellMax">-- V</div>
                    <div class="cell-stat-label">Max</div>
                  </div>
                  <div class="cell-stat">
                    <div class="cell-stat-value min" id="cellMin">-- V</div>
                    <div class="cell-stat-label">Min</div>
                  </div>
                </div>
              </div>
              <div class="cells-grid" id="cellsGrid">
                ${Array.from({length: cellCount}, (_, i) => `
                  <div class="cell" id="cell${i + 1}">
                    <div class="cell-num">${i + 1}</div>
                    <div class="cell-voltage" id="cellVoltage${i + 1}">--</div>
                    <div class="cell-bar">
                      <div class="cell-bar-fill" id="cellBar${i + 1}" style="width: 0%"></div>
                    </div>
                  </div>
                `).join('')}
              </div>
            </div>
            ` : ''}

            ${config.show_energy ? `
            <!-- Energy Section -->
            <div class="energy-section">
              <div class="energy-item">
                <div class="energy-icon">⚡</div>
                <div class="energy-value" id="energyIn">-- kWh</div>
                <div class="energy-label">Energy In</div>
              </div>
              <div class="energy-item">
                <div class="energy-icon">🔋</div>
                <div class="energy-value" id="energyOut">-- kWh</div>
                <div class="energy-label">Energy Out</div>
              </div>
              <div class="energy-item">
                <div class="energy-icon">🔄</div>
                <div class="energy-value" id="cycleCount">--</div>
                <div class="energy-label">Cycles</div>
              </div>
              <div class="energy-item">
                <div class="energy-icon">📊</div>
                <div class="energy-value" id="cycleCapacity">-- Ah</div>
                <div class="energy-label">Cycle Cap</div>
              </div>
            </div>
            ` : ''}

            ${config.show_balance ? `
            <!-- Balance Indicator -->
            <div class="balance-indicator">
              <span class="balance-dot" id="balanceDot"></span>
              <span class="balance-text" id="balanceText">Balance: --</span>
            </div>
            ` : ''}
          </div>
        </div>
      </ha-card>
    `;
  }

  updateValues() {
    if (!this._hass || !this.shadowRoot) return;
    const config = this._config;

    // Check if primary entity is available
    const socEntity = config.soc_entity;
    const isOnline = this.isAvailable(socEntity);
    
    const statusDot = this.shadowRoot.getElementById('statusDot');
    const statusText = this.shadowRoot.getElementById('statusText');
    const container = this.shadowRoot.querySelector('.card-container');
    
    if (statusDot && statusText) {
      if (isOnline) {
        statusDot.classList.remove('offline');
        statusText.textContent = 'ONLINE';
        container?.classList.remove('unavailable');
      } else {
        statusDot.classList.add('offline');
        statusText.textContent = 'OFFLINE';
        container?.classList.add('unavailable');
      }
    }

    // SOC
    const soc = this.getState(config.soc_entity);
    const socProgress = this.shadowRoot.getElementById('socProgress');
    const socValue = this.shadowRoot.getElementById('socValue');
    if (socProgress && soc !== null) {
      const circumference = 2 * Math.PI * 60;
      const offset = circumference - (soc / 100) * circumference;
      socProgress.style.strokeDashoffset = offset;
    }
    if (socValue) {
      socValue.textContent = soc !== null ? `${Math.round(soc)}%` : '--%';
    }

    // Voltage
    const voltage = this.getState(config.voltage_entity);
    const voltageValue = this.shadowRoot.getElementById('voltageValue');
    const voltagePanel = this.shadowRoot.getElementById('voltagePanel');
    const voltageBar = this.shadowRoot.getElementById('voltageBar');
    if (voltageValue) {
      voltageValue.textContent = voltage !== null ? `${voltage.toFixed(1)} V` : '-- V';
    }
    if (voltagePanel) {
      voltagePanel.textContent = voltage !== null ? voltage.toFixed(1) : '--';
    }
    if (voltageBar && voltage !== null) {
      const voltagePercent = ((voltage - 40) / (58.4 - 40)) * 100;
      voltageBar.style.width = `${Math.min(100, Math.max(0, voltagePercent))}%`;
    }

    // Current
    const current = this.getState(config.current_entity);
    const currentValue = this.shadowRoot.getElementById('currentValue');
    if (currentValue) {
      if (current !== null) {
        currentValue.textContent = `${Math.abs(current).toFixed(2)} A`;
        currentValue.className = current > 0 ? 'stat-value charging' : 'stat-value discharging';
      } else {
        currentValue.textContent = '-- A';
        currentValue.className = 'stat-value';
      }
    }

    // Power
    const power = this.getState(config.power_entity);
    const powerValue = this.shadowRoot.getElementById('powerValue');
    if (powerValue) {
      if (power !== null) {
        powerValue.textContent = `${Math.abs(power).toFixed(0)} W`;
        powerValue.className = power > 0 ? 'stat-value charging' : 'stat-value discharging';
      } else {
        powerValue.textContent = '-- W';
        powerValue.className = 'stat-value';
      }
    }

    // Capacity
    const capFull = this.getState(config.capacity_full_entity);
    const capRemaining = this.getState(config.capacity_remaining_entity);
    const capacityValue = this.shadowRoot.getElementById('capacityValue');
    if (capacityValue) {
      const remaining = capRemaining !== null ? capRemaining.toFixed(1) : '--';
      const full = capFull !== null ? capFull.toFixed(0) : '--';
      capacityValue.textContent = `${remaining} / ${full} Ah`;
    }

    // SOH
    const soh = this.getState(config.soh_entity);
    const sohValue = this.shadowRoot.getElementById('sohValue');
    const sohBar = this.shadowRoot.getElementById('sohBar');
    if (sohValue) {
      sohValue.textContent = soh !== null ? Math.round(soh) : '--';
    }
    if (sohBar && soh !== null) {
      sohBar.style.width = `${soh}%`;
    }

    // Temperatures
    if (config.show_temps) {
      const battTemp = this.getState(config.battery_temp_entity);
      const mosTemp = this.getState(config.mos_temp_entity);
      const battTempValue = this.shadowRoot.getElementById('battTempValue');
      const mosTempValue = this.shadowRoot.getElementById('mosTempValue');
      if (battTempValue) {
        battTempValue.textContent = battTemp !== null ? `${battTemp.toFixed(1)}°F` : '--°F';
      }
      if (mosTempValue) {
        mosTempValue.textContent = mosTemp !== null ? `${mosTemp.toFixed(1)}°F` : '--°F';
      }
    }

    // Cell voltages
    if (config.show_cells) {
      const cellDelta = this.getState(config.cell_delta_entity);
      const cellMax = this.getState(config.cell_max_entity);
      const cellMin = this.getState(config.cell_min_entity);

      const cellDeltaEl = this.shadowRoot.getElementById('cellDelta');
      const cellMaxEl = this.shadowRoot.getElementById('cellMax');
      const cellMinEl = this.shadowRoot.getElementById('cellMin');

      if (cellDeltaEl) {
        cellDeltaEl.textContent = cellDelta !== null ? `${cellDelta.toFixed(1)} mV` : '-- mV';
      }
      if (cellMaxEl) {
        cellMaxEl.textContent = cellMax !== null ? `${cellMax.toFixed(3)} V` : '-- V';
      }
      if (cellMinEl) {
        cellMinEl.textContent = cellMin !== null ? `${cellMin.toFixed(3)} V` : '-- V';
      }

      // Individual cells
      const cellCount = config.cell_count || 16;
      const cellPrefix = config.cell_prefix;
      const cellVoltages = [];

      for (let i = 1; i <= cellCount; i++) {
        const cellEntity = cellPrefix ? `${cellPrefix}${i}` : null;
        const cellVoltage = this.getState(cellEntity);
        cellVoltages.push(cellVoltage);

        const cellVoltageEl = this.shadowRoot.getElementById(`cellVoltage${i}`);
        const cellBarEl = this.shadowRoot.getElementById(`cellBar${i}`);
        const cellEl = this.shadowRoot.getElementById(`cell${i}`);

        if (cellVoltageEl) {
          cellVoltageEl.textContent = cellVoltage !== null ? cellVoltage.toFixed(3) : '--';
        }
        if (cellBarEl && cellVoltage !== null) {
          const barPercent = ((cellVoltage - 2.5) / (3.65 - 2.5)) * 100;
          cellBarEl.style.width = `${Math.min(100, Math.max(0, barPercent))}%`;
        }
      }

      // Highlight min/max cells
      const validVoltages = cellVoltages.filter(v => v !== null);
      if (validVoltages.length > 0) {
        const minV = Math.min(...validVoltages);
        const maxV = Math.max(...validVoltages);

        for (let i = 1; i <= cellCount; i++) {
          const cellEl = this.shadowRoot.getElementById(`cell${i}`);
          if (cellEl) {
            cellEl.classList.remove('high', 'low');
            if (cellVoltages[i - 1] === maxV) {
              cellEl.classList.add('high');
            } else if (cellVoltages[i - 1] === minV) {
              cellEl.classList.add('low');
            }
          }
        }
      }
    }

    // Energy stats
    if (config.show_energy) {
      const energyIn = this.getState(config.energy_in_entity);
      const energyOut = this.getState(config.energy_out_entity);
      const cycleCount = this.getState(config.cycle_count_entity);
      const cycleCapacity = this.getState(config.cycle_capacity_entity);

      const energyInEl = this.shadowRoot.getElementById('energyIn');
      const energyOutEl = this.shadowRoot.getElementById('energyOut');
      const cycleCountEl = this.shadowRoot.getElementById('cycleCount');
      const cycleCapacityEl = this.shadowRoot.getElementById('cycleCapacity');

      if (energyInEl) {
        energyInEl.textContent = energyIn !== null ? `${Math.abs(energyIn).toFixed(1)} kWh` : '-- kWh';
      }
      if (energyOutEl) {
        energyOutEl.textContent = energyOut !== null ? `${Math.abs(energyOut).toFixed(1)} kWh` : '-- kWh';
      }
      if (cycleCountEl) {
        cycleCountEl.textContent = cycleCount !== null ? Math.round(cycleCount) : '--';
      }
      if (cycleCapacityEl) {
        cycleCapacityEl.textContent = cycleCapacity !== null ? `${cycleCapacity.toFixed(0)} Ah` : '-- Ah';
      }
    }

    // Balance indicator
    if (config.show_balance) {
      const balanceCurrent = this.getState(config.balance_current_entity);
      const balanceDot = this.shadowRoot.getElementById('balanceDot');
      const balanceText = this.shadowRoot.getElementById('balanceText');

      if (balanceDot && balanceText) {
        if (balanceCurrent !== null && balanceCurrent > 0) {
          balanceDot.classList.add('active');
          balanceText.textContent = `Balancing: ${balanceCurrent} mA`;
        } else {
          balanceDot.classList.remove('active');
          balanceText.textContent = balanceCurrent !== null ? 'Balance: IDLE' : 'Balance: --';
        }
      }
    }
  }

  getCardSize() {
    return this._config.compact ? 4 : 6;
  }
}

// Register elements
customElements.define('jk-bms-card-editor', JKBMSCardEditor);
customElements.define('jk-bms-card', JKBMSCard);

// Register with Home Assistant
window.customCards = window.customCards || [];
window.customCards.push({
  type: 'jk-bms-card',
  name: 'JK BMS Power Core',
  description: 'A high-tech battery monitoring card for JK BMS systems',
  preview: true,
  documentationURL: 'https://github.com/your-repo/jk-bms-card',
});
