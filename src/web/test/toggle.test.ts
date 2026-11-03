import assert from 'node:assert/strict';
import { test } from 'node:test';
import { RemoteToggle } from '../src/toggle.ts';

function fakeInput() {
  let onChange = () => {};
  const classes = new Set<string>();
  const input = {
    checked: false,
    disabled: false,
    classes,
    classList: { toggle: (t: string, f?: boolean) => (f ? classes.add(t) : classes.delete(t)) },
    addEventListener: (_: 'change', l: () => void) => (onChange = l),
    click(value: boolean) {
      input.checked = value;
      onChange();
    },
  };
  return input;
}

test('toggle follows telemetry and is disabled offline', () => {
  const input = fakeInput();
  const t = new RemoteToggle(input, () => {}, 1000, () => 0);
  assert.equal(input.disabled, true);
  t.sync(true);
  assert.equal(input.disabled, false);
  assert.equal(input.checked, true);
  t.sync(false, false);
  assert.equal(input.checked, false);
  assert.equal(input.disabled, true);
});

test('a click stays pending until telemetry confirms it', () => {
  let now = 0;
  const sent: boolean[] = [];
  const input = fakeInput();
  const t = new RemoteToggle(input, (on) => sent.push(on), 1000, () => now);
  t.sync(false);
  input.click(true);
  assert.deepEqual(sent, [true]);
  now = 500;
  t.sync(false); // not applied yet: keep showing the request
  assert.equal(input.checked, true);
  assert.ok(input.classes.has('pending'));
  t.sync(true);
  assert.equal(input.checked, true);
  assert.ok(!input.classes.has('pending'));
});

test('an unconfirmed click falls back to the robot state after the timeout', () => {
  let now = 0;
  const input = fakeInput();
  const t = new RemoteToggle(input, () => {}, 1000, () => now);
  t.sync(false);
  input.click(true);
  now = 1500;
  t.sync(false);
  assert.equal(input.checked, false);
  assert.ok(!input.classes.has('pending'));
});
