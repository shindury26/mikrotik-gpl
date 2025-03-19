#ifndef STACK_H
#define STACK_H


#define STATS_STACK_SIZE 30
struct stack {
    unsigned data[STATS_STACK_SIZE];
    unsigned sp;
};

void stack_init(struct stack *s) {
    memset(s, 0, sizeof(*s));
}

void stack_push(struct stack *s, unsigned v) {
    if (s->sp >= STATS_STACK_SIZE) {
	return;
    }
    s->data[s->sp] = v;
    ++s->sp;
}

void stack_dump(struct stack *s) {
    unsigned i;
    printk("stack size %u\n", s->sp);
    for (i = 0; i < s->sp; ++i) {
	if (i && !(i % 10)) printk("\n");
	if (!(i % 10)) printk("% 3d:", i);
	printk(" % 4d", s->data[i]);
    }
    printk("\n");
}

#endif
