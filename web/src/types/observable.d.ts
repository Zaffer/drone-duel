// Chrome's native Observables (WICG Observable, shipped in Chrome 135), which TypeScript's DOM lib
// does not declare yet: the subset the game uses (DESIGN.md §9).

interface SubscriptionObserver<T> {
  next?: (value: T) => void;
  error?: (error: unknown) => void;
  complete?: () => void;
}

interface SubscribeOptions {
  signal?: AbortSignal;
}

interface Subscriber<T> {
  next(value: T): void;
  error(error: unknown): void;
  complete(): void;
  addTeardown(teardown: () => void): void;
  readonly active: boolean;
  readonly signal: AbortSignal;
}

type ObservableInput<T> = Observable<T> | AsyncIterable<T> | Iterable<T> | Promise<T>;

declare class Observable<T> {
  constructor(subscribe: (subscriber: Subscriber<T>) => void);
  static from<T>(value: ObservableInput<T>): Observable<T>;
  subscribe(
    observer?: SubscriptionObserver<T> | ((value: T) => void),
    options?: SubscribeOptions,
  ): void;

  takeUntil(notifier: ObservableInput<unknown>): Observable<T>;
  map<U>(mapper: (value: T, index: number) => U): Observable<U>;
  filter<S extends T>(predicate: (value: T, index: number) => value is S): Observable<S>;
  filter(predicate: (value: T, index: number) => boolean): Observable<T>;
  take(amount: number): Observable<T>;
  drop(amount: number): Observable<T>;
  flatMap<U>(mapper: (value: T, index: number) => ObservableInput<U>): Observable<U>;
  switchMap<U>(mapper: (value: T, index: number) => ObservableInput<U>): Observable<U>;
  inspect(observer?: SubscriptionObserver<T> | ((value: T) => void)): Observable<T>;
  catch<U>(handler: (error: unknown) => ObservableInput<U>): Observable<T | U>;
  finally(callback: () => void): Observable<T>;

  toArray(options?: SubscribeOptions): Promise<T[]>;
  forEach(callback: (value: T, index: number) => void, options?: SubscribeOptions): Promise<void>;
  first(options?: SubscribeOptions): Promise<T>;
  last(options?: SubscribeOptions): Promise<T>;
  find(
    predicate: (value: T, index: number) => boolean,
    options?: SubscribeOptions,
  ): Promise<T | undefined>;
  some(
    predicate: (value: T, index: number) => boolean,
    options?: SubscribeOptions,
  ): Promise<boolean>;
  every(
    predicate: (value: T, index: number) => boolean,
    options?: SubscribeOptions,
  ): Promise<boolean>;
  reduce<A>(
    reducer: (acc: A, value: T, index: number) => A,
    initial: A,
    options?: SubscribeOptions,
  ): Promise<A>;
}

interface ObservableEventListenerOptions {
  capture?: boolean;
  passive?: boolean;
}

interface EventTarget {
  when<E extends Event = Event>(
    type: string,
    options?: ObservableEventListenerOptions,
  ): Observable<E>;
}
