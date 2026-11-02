const MAX_OUTPUT_CHARS = 30000;
const MAX_HISTORY = 50;

/** Terminal for the robot's CLI: plain-text output and command history. */
export class Cli {
  private history: string[] = [];
  private historyPos = 0;

  private readonly output: HTMLElement;
  private readonly input: HTMLInputElement;
  private readonly send: (command: string) => boolean;

  constructor(output: HTMLElement, input: HTMLInputElement, form: HTMLFormElement, send: (command: string) => boolean) {
    this.output = output;
    this.input = input;
    this.send = send;
    form.addEventListener('submit', (e) => {
      e.preventDefault();
      this.submit();
    });
    input.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowUp') this.recall(-1, e);
      if (e.key === 'ArrowDown') this.recall(1, e);
    });
  }

  write(text: string): void {
    const atBottom = this.output.scrollHeight - this.output.scrollTop - this.output.clientHeight < 8;
    // textContent, never innerHTML: robot output is not markup
    let content = (this.output.textContent ?? '') + text;
    if (content.length > MAX_OUTPUT_CHARS) content = content.slice(-MAX_OUTPUT_CHARS);
    this.output.textContent = content;
    if (atBottom) this.output.scrollTop = this.output.scrollHeight;
  }

  focus(): void {
    this.input.focus();
  }

  private submit(): void {
    const command = this.input.value;
    if (!this.send(command)) {
      this.write('[offline] command not sent\n');
      return;
    }
    if (command && command !== this.history[this.history.length - 1]) {
      this.history.push(command);
      if (this.history.length > MAX_HISTORY) this.history.shift();
    }
    this.historyPos = this.history.length;
    this.input.value = '';
    this.input.focus();
  }

  private recall(step: number, e: KeyboardEvent): void {
    if (!this.history.length) return;
    e.preventDefault();
    this.historyPos = Math.min(this.history.length, Math.max(0, this.historyPos + step));
    this.input.value = this.history[this.historyPos] ?? '';
  }
}
