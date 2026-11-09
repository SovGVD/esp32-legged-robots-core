/**
 * Settings tab: the robot's CLI settings as switches, buttons and pictures,
 * grouped by topic. The robot reports its state in the setup package ('S',
 * asked for a few times a second while this tab is open) and every change goes
 * back as the same CLI command you would type (`terrain on`, `heading cal`,
 * `trim LF alpha 1.5`), so the robot's replies show in the log at the bottom.
 */
import type { Config } from './config.ts';
import { correct, drawDial, drawLevel, drawMagPlot, dialAngle, roundness, type P2 } from './compass.ts';
import { CAL_STATES, SH, SO, TS, type SetupState, type Telemetry } from './protocol.ts';
import { RemoteToggle } from './toggle.ts';

const JOINTS = ['alpha', 'beta', 'gamma', 'delta', 'epsilon', 'zeta'];
const MAX_POINTS = 720;
const SWITCH_MS = 15000; // terrain and feet switch once the robot stands still
const TRIM_MS = 3000;
const LOG_LINES = 14;

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls = '', text = ''): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text) e.textContent = text;
  return e;
}

function btn(text: string, click: () => void, cls = ''): HTMLButtonElement {
  const b = el('button', cls, text);
  b.type = 'button';
  b.addEventListener('click', click);
  return b;
}

/** A button that asks "sure?" on the first tap (no dialogs: they are blocked in some embeds). */
function sureBtn(text: string, click: () => void): HTMLButtonElement {
  let armed = 0;
  const b = btn(text, () => {
    if (Date.now() - armed < 3000) {
      armed = 0;
      b.textContent = text;
      click();
      return;
    }
    armed = Date.now();
    b.textContent = 'Tap again';
    setTimeout(() => (b.textContent = text), 3000);
  });
  return b;
}

function title(b: string, d: string): HTMLSpanElement {
  const s = el('span');
  s.append(el('b', '', b), el('span', 'dim', d));
  return s;
}

function row(b: string, d: string, ...right: HTMLElement[]): HTMLDivElement {
  const r = el('div', 'row');
  const tools = el('span', 'tools');
  tools.append(...right);
  r.append(title(b, d), tools);
  return r;
}

function switchRow(b: string, d: string): [HTMLLabelElement, HTMLInputElement] {
  const r = el('label', 'row');
  const i = el('input');
  i.type = 'checkbox';
  r.append(i, title(b, d));
  return [r, i];
}

interface Section {
  key: string;
  name: string;
  box: HTMLElement;
  show: () => boolean;
}

interface Switch {
  toggle: RemoteToggle;
  on: (s: SetupState) => boolean;
  can: (s: SetupState) => boolean;
  row: HTMLElement;
  has: (s: SetupState) => boolean;
}

export class SettingsPanel {
  private state: SetupState | null = null;
  private tele: Telemetry | null = null;
  private visible = false;
  private section = '';
  private sections: Section[] = [];
  private readonly nav: HTMLElement;
  private readonly logBox: HTMLPreElement;
  private readonly switches: Switch[] = [];
  private shape = '';

  // compass
  private points: P2[] = [];
  private corrected = false;
  private calWas = 0;
  private turnTarget: { deg: number; at: number } | null = null;
  private readonly dial = el('canvas', 'plot dial');
  private readonly plot = el('canvas', 'plot');
  private readonly level = el('canvas', 'plot level');
  private readonly headingText = el('p', 'small dim center');
  private readonly calText = el('p', 'small center');
  private readonly calStats = el('p', 'small dim mono center');
  private readonly calBox = el('div');
  private readonly noMag = el('p', 'small dim', 'No magnetometer (an MPU6500 board?): the heading is the gyro only, relative to where the robot started.');
  private readonly plotSeg = el('span', 'seg wide');

  // sensors
  private readonly tofChips = el('span', 'chips');
  private readonly feetChips = el('span', 'chips');
  private feetLive: HTMLElement;
  private readonly imuText = el('p', 'small dim center');

  // servos
  private leg = 0;
  private readonly legSeg = el('span', 'seg wide');
  private readonly jointRows = el('div');
  private readonly trimPending = new Map<string, { v: number; at: number }>();
  private dirty = false;
  private readonly saveRow: HTMLElement;
  private calPose = false;
  private readonly poseInput: HTMLInputElement;

  private readonly send: (cmd: string) => void;
  private readonly config: Config;

  constructor(root: HTMLElement, config: Config, send: (cmd: string) => void) {
    this.send = send;
    this.config = config;
    this.nav = root.querySelector('#set-nav')!;
    this.logBox = root.querySelector('#set-log')!;
    const adopt = (key: string) => root.querySelector<HTMLElement>(`[data-sec="${key}"]`)!;
    const add = (key: string, name: string, show: () => boolean, box: HTMLElement = el('div')): HTMLElement => {
      box.dataset.sec = key;
      this.sections.push({ key, name, box, show });
      return box;
    };
    const s = () => this.state;

    // ---- Move: what the legs do
    const move = add('move', 'Move', () => true, adopt('move'));
    const hal = this.addSwitch('Leg servos', 'Off: the servos stop following the gait and keep their last position.', 'hal', (st) => !!(st.has & SH.hal));
    move.prepend(hal);
    const terrain = this.addSwitch('Terrain mode', 'Rough ground: ToF height map, slower rough gait, leveling, stops before gaps. Switches when the robot stands still.', 'terrain', (st) => !!(st.on & SO.terrain), (st) => !!(st.has & SH.terrain), (st) => !!(st.has & SH.imu));
    const feet = this.addSwitch('Foot switches', 'A step ends when its foot touches the ground, instead of on time. Switches when the robot stands still.', 'feet', (st) => !!(st.on & SO.feet), (st) => !!(st.has & SH.feet));
    this.feetLive = row('Feet now', 'Lit = pressed. Press each foot by hand to test the wiring.', this.feetChips);
    move.querySelector('#rotate')?.closest('.row')?.before(terrain, feet, this.feetLive);

    // ---- Compass: heading hold, turn to, magnetometer calibration
    const compass = add('compass', 'Compass', () => !!s() && !!(s()!.has & SH.imu));
    compass.append(this.addSwitch('Heading hold', 'Keep the heading when you let go of the turn stick; tap the dial to turn to a heading.', 'heading', (st) => !!(st.on & SO.hold)));
    this.dial.addEventListener('pointerdown', (e) => {
      const r = this.dial.getBoundingClientRect();
      const deg = Math.round(dialAngle(e.clientX - r.left - r.width / 2, e.clientY - r.top - r.height / 2) / 5) * 5 % 360;
      this.turnTarget = { deg, at: Date.now() };
      this.send(`turnto ${deg}`);
      this.draw();
    });
    compass.append(this.dial, this.headingText);
    const h4 = el('h4', '', 'Magnetometer calibration');
    const help = el('p', 'small dim', 'Calibrate on flat ground away from steel and cables. The robot turns two full turns by itself (keep this page open), or turn it slowly by hand within 60 s. The dots are the readings seen from above: before calibration they sit off centre and squashed; corrected they should form a thin ring round the middle.');
    for (const [k, t] of [['raw', 'Raw'], ['fixed', 'Corrected']]) {
      this.plotSeg.append(btn(t, () => {
        this.corrected = k === 'fixed';
        this.draw();
      }));
    }
    this.plotSeg.dataset.k = 'plot';
    this.calBox.append(
      this.plotSeg,
      this.plot,
      this.calText,
      this.calStats,
      row('Calibrate', 'Saved on the robot when the two turns are done.', btn('Turn by itself', () => this.calibrate('cal')), btn('By hand', () => this.calibrate('hand'))),
      row('Readings', 'Collected while this tab is open.', btn('Clear', () => {
        this.points = [];
        this.draw();
      })),
    );
    compass.append(h4, help, this.noMag, this.calBox);

    // ---- Sensors: IMU, ToF
    add('sensors', 'Sensors', () => true).append(
      el('h4', '', 'IMU'),
      this.level,
      this.imuText,
      row('Calibrate IMU', 'Put the robot on a flat level surface and keep it still: takes 5 s, the servos relax meanwhile.', sureBtn('Calibrate', () => this.send('calib imu'))),
      ...(config.tofSensors
        ? [el('h4', '', 'ToF sensors'), row('Running', 'VL53L5CX 8x8 distance sensors.', this.tofChips), row('Zones', 'Print the 64 distances of each sensor to the log.', btn('Print', () => this.send('tof print')))]
        : []),
    );

    // ---- Servos: trims
    this.poseInput = el('input');
    this.poseInput.type = 'checkbox';
    const pose = el('label', 'row');
    pose.append(this.poseInput, title('Calibration pose', 'Leg servos off the gait, every joint at its calibration angle: trim each joint until the leg matches it.'));
    this.poseInput.addEventListener('change', () => {
      this.calPose = this.poseInput.checked;
      this.send(this.calPose ? 'hal off' : 'hal on');
      if (this.calPose) this.send('calib leg');
    });
    this.saveRow = row('Save trims', 'Until saved, the trims are lost on reboot.', btn('Save', () => {
      this.send('save');
      this.dirty = false;
      this.renderTrims();
    }));
    add('servos', 'Servos', () => !!s() && s()!.dof > 0).append(
      pose,
      el('h4', '', 'Trim'),
      this.legSeg,
      this.jointRows,
      this.saveRow,
    );

    // ---- Map
    add('map', 'Map', () => config.lidar).append(
      row('Walking to a goal', 'Stop walking to the point picked on the map.', btn('Stop', () => this.send('lidar stop'))),
      row('Freeze map', 'Stop adding to the map (keep localizing in it). Tap again to unfreeze.', btn('Freeze', () => this.send('lidar freeze'))),
      row('Clear map', 'Start a new map: the robot is at 0, 0 facing forward.', sureBtn('Clear', () => this.send('lidar reset'))),
    );

    // ---- Info: read-only CLI reports
    const info = add('info', 'Info', () => true);
    info.append(el('p', 'small dim', 'Ask the robot for a report, the answer shows in the log below.'));
    const reports = el('div', 'buttons');
    const report = (t: string, cmd: string, when: () => boolean = () => true) => {
      const b = btn(t, () => this.send(cmd));
      b.dataset.when = String(this.reports.length);
      this.reports.push(when);
      reports.append(b);
    };
    report('Settings', 'show');
    report('Gait', 'gait state', () => !!s() && !!(s()!.has & SH.walk));
    report('Loop timing', 'loop');
    report('Leveling', 'level state');
    report('Heading', 'heading state', () => !!s() && !!(s()!.has & SH.imu));
    report('Terrain', 'terrain state', () => !!s() && !!(s()!.has & SH.terrain));
    report('ToF', 'tof state', () => config.tofSensors > 0);
    report('Feet', 'feet state', () => !!s() && !!(s()!.has & SH.feet));
    report('Map', 'lidar state', () => config.lidar);
    report('WiFi', 'wifi');
    report('HIL link', 'hil', () => !!s() && !!(s()!.has & SH.hil));
    report('All commands', 'help');
    info.append(reports);

    add('app', 'App', () => true, adopt('app'));

    const log = this.logBox;
    this.sections.forEach((sec) => log.before(sec.box));
    this.section = this.load() ?? 'move';
    this.renderNav();
    this.renderTrims();
  }

  private readonly reports: (() => boolean)[] = [];

  private addSwitch(
    b: string,
    d: string,
    cmd: string,
    on: (s: SetupState) => boolean,
    has: (s: SetupState) => boolean = () => true,
    can: (s: SetupState) => boolean = () => true,
  ): HTMLLabelElement {
    const [r, i] = switchRow(b, d);
    const toggle = new RemoteToggle(i, (v) => this.send(`${cmd} ${v ? 'on' : 'off'}`), SWITCH_MS);
    this.switches.push({ toggle, on, can, row: r, has });
    return r;
  }

  private load(): string | null {
    try {
      return localStorage.getItem('settings-sec');
    } catch {
      return null;
    }
  }

  private pick(key: string): void {
    this.section = key;
    try {
      localStorage.setItem('settings-sec', key);
    } catch {
      /* not remembered */
    }
    this.renderNav();
    this.draw();
  }

  private renderNav(): void {
    const shown = this.sections.filter((s) => s.show());
    if (!shown.some((s) => s.key === this.section)) this.section = 'move';
    this.nav.replaceChildren(
      ...shown.map((s) => {
        const b = btn(s.name, () => this.pick(s.key));
        b.classList.toggle('on', s.key === this.section);
        return b;
      }),
    );
    this.sections.forEach((s) => (s.box.hidden = s.key !== this.section));
    this.nav.parentElement!.querySelectorAll<HTMLButtonElement>('[data-when]').forEach((b) => {
      b.hidden = !this.reports[Number(b.dataset.when)]();
    });
  }

  show(visible: boolean): void {
    this.visible = visible;
    this.draw();
  }

  get active(): string {
    return this.section;
  }

  /** CLI output: the last lines show under the settings. */
  log(text: string): void {
    const lines = ((this.logBox.textContent ?? '') + text).split('\n');
    this.logBox.textContent = lines.slice(-LOG_LINES).join('\n');
  }

  private calibrate(mode: 'cal' | 'hand'): void {
    this.points = [];
    this.send(`heading ${mode}`);
    this.draw();
  }

  setTelemetry(t: Telemetry | null): void {
    this.tele = t;
    if (this.visible && (this.section === 'compass' || this.section === 'sensors')) this.draw();
  }

  setState(st: SetupState | null): void {
    const prev = this.state;
    this.state = st;
    for (const sw of this.switches) {
      sw.row.hidden = !!st && !sw.has(st);
      sw.toggle.sync(st ? sw.on(st) : null, !!st && sw.can(st));
    }
    if (!st) {
      this.renderNav();
      return;
    }
    // new robot shape (first state, or a reboot into another model): rebuild the lists
    const shape = `${st.has & ~(SH.hal | SH.raw)}|${st.dof}|${st.legs.map((l) => l.name).join(',')}`;
    if (shape !== this.shape) {
      this.shape = shape;
      this.renderNav();
      this.feetChips.replaceChildren(...st.legs.map((l) => el('span', 'chip', l.name)));
      this.tofChips.replaceChildren(...Array.from({ length: this.config.tofSensors }, (_, i) => el('span', 'chip', `ToF ${i + 1}`)));
      this.legSeg.replaceChildren(
        ...st.legs.map((l, i) => btn(l.name, () => {
          this.leg = i;
          this.renderTrims();
        })),
      );
    }
    [...this.feetChips.children].forEach((c, i) => c.classList.toggle('on', !!(st.feet & (1 << i))));
    [...this.tofChips.children].forEach((c, i) => c.classList.toggle('on', !!(st.tof & (1 << i))));

    // a calibration starting on the robot (from here or the CLI): start a fresh cloud
    const calibrating = st.cal.state === 1 || st.cal.state === 2;
    if (calibrating && !(this.calWas === 1 || this.calWas === 2)) this.points = [];
    this.calWas = st.cal.state;
    const raw = st.mag.raw;
    if (raw) {
      const last = this.points[this.points.length - 1];
      if (!last || last[0] !== raw[0] || last[1] !== raw[1]) this.points.push([raw[0], raw[1]]);
      if (this.points.length > MAX_POINTS) this.points.shift();
    }
    if (this.calPose && st.has & SH.hal && prev && !(prev.has & SH.hal)) {
      // leg servos switched back on (Move tab, CLI): no longer in the calibration pose
      this.calPose = false;
    }
    this.feetLive.hidden = !(st.has & SH.feet);
    this.poseInput.checked = this.calPose;
    this.renderTrims();
    this.draw();
  }

  // ---- servo trims

  private trimValue(leg: number, j: number): number {
    const p = this.trimPending.get(`${leg}.${j}`);
    const actual = this.state?.legs[leg]?.trim[j] ?? 0;
    if (p && (Math.abs(p.v - actual) < 0.006 || Date.now() - p.at > TRIM_MS)) this.trimPending.delete(`${leg}.${j}`);
    return this.trimPending.get(`${leg}.${j}`)?.v ?? actual;
  }

  private nudge(j: number, step: number): void {
    const st = this.state;
    const leg = st?.legs[this.leg];
    if (!leg) return;
    const v = Math.round((this.trimValue(this.leg, j) + step) * 100) / 100;
    this.trimPending.set(`${this.leg}.${j}`, { v, at: Date.now() });
    this.send(`trim ${leg.name} ${JOINTS[j]} ${v.toFixed(2)}`);
    if (this.calPose) this.send('calib leg'); // trims apply when the pose is written again
    this.dirty = true;
    this.renderTrims();
  }

  private renderTrims(): void {
    const st = this.state;
    [...this.legSeg.children].forEach((b, i) => b.classList.toggle('on', i === this.leg));
    this.saveRow.classList.toggle('warn', this.dirty);
    if (!st || !st.legs[this.leg]) {
      this.jointRows.replaceChildren();
      return;
    }
    if (this.jointRows.childElementCount !== st.dof) {
      this.jointRows.replaceChildren(
        ...Array.from({ length: st.dof }, (_, j) => {
          const v = el('span', 'mono trim');
          const r = row(JOINTS[j], '', btn('−1', () => this.nudge(j, -1)), btn('−.1', () => this.nudge(j, -0.1)), v, btn('+.1', () => this.nudge(j, 0.1)), btn('+1', () => this.nudge(j, 1)));
          r.classList.add('joint');
          return r;
        }),
      );
    }
    [...this.jointRows.children].forEach((r, j) => {
      const v = r.querySelector('.trim')!;
      v.textContent = `${this.trimValue(this.leg, j).toFixed(2)}°`;
      v.classList.toggle('pending', this.trimPending.has(`${this.leg}.${j}`));
    });
  }

  // ---- drawings

  private draw(): void {
    if (!this.visible) return;
    [...this.plotSeg.children].forEach((b, i) => b.classList.toggle('on', i === (this.corrected ? 1 : 0)));
    const st = this.state;
    const t = this.tele;
    if (this.section === 'compass') {
      const heading = t?.heading ?? null;
      const compass = !!t && !!(t.statuses & TS.headingAbsolute);
      let target = t && t.statuses & TS.headingHold ? t.headingTarget ?? null : null;
      if (this.turnTarget && Date.now() - this.turnTarget.at < 2500 && target === null) target = this.turnTarget.deg;
      drawDial(this.dial, heading, target, compass);
      this.headingText.textContent = heading === null ? 'No heading' : `${compass ? 'Compass heading' : 'Turned since start (gyro only)'}${target !== null ? ` · holding ${Math.round(target)}°` : ''}`;
      const mag = !!st && !!(st.has & SH.mag);
      this.noMag.hidden = mag;
      this.calBox.hidden = !mag;
      if (!st || !mag) return;
      const { cal, mag: m } = st;
      const busy = cal.state === 1 || cal.state === 2;
      drawMagPlot(this.plot, {
        points: this.points,
        corrected: this.corrected,
        offset: m.offset,
        scale: m.scale,
        calibrated: m.calField > 0,
        progress: busy ? cal.turned / 720 : null,
      });
      this.calText.textContent = busy
        ? `${cal.state === 1 ? 'Turning' : 'Turn it slowly by hand'}: ${cal.turned}° of 720°, ${cal.time} s`
        : `Calibration: ${m.calField > 0 ? 'saved' : 'not calibrated'}${cal.state >= 3 ? ` (last run ${CAL_STATES[cal.state]})` : ''}`;
      this.calText.classList.toggle('warn', cal.state === 4 || cal.state === 5);
      this.calStats.textContent =
        `field ${m.field.toFixed(1)} µT ${st.on & SO.magTrusted ? 'used' : 'ignored'}` +
        (m.calField > 0 ? ` · offset ${m.offset[0].toFixed(1)}, ${m.offset[1].toFixed(1)} · scale ${m.scale[0].toFixed(2)}, ${m.scale[1].toFixed(2)}` : '');
      const rd = roundness(this.points.map((q) => correct(q, m.offset, m.scale)));
      if (rd && m.calField > 0) this.calStats.textContent += ` · ring ±${Math.round(rd.spread * 100)}% ${rd.spread < 0.08 ? '(good)' : '(recalibrate?)'}`;
    } else if (this.section === 'sensors') {
      const imu = !!t && !!(t.statuses & TS.imu) && t.pitch !== undefined;
      drawLevel(this.level, imu ? t!.pitch! : null, imu ? t!.roll! : null);
      this.imuText.textContent = imu ? `nose ${t!.roll!.toFixed(1)}° · sideways ${t!.pitch!.toFixed(1)}° (dot in the middle = level)` : 'No IMU';
    }
  }
}
