#ifndef __SW_LIST_H__
#define __SW_LIST_H__

#include <linux/list.h>

#define SW_LIST_HEAD(name, dummy)                   struct list_head name
#define SW_LIST_ENTRY(name, dummy)                  struct list_head name
#define SW_LIST_HEAD_VAR(dummy)                     struct list_head
#define SW_LIST_HEAD_INIT(head)                     INIT_LIST_HEAD(head)
#define SW_LIST_ENTRY_INIT(node, field)             INIT_LIST_HEAD(&node->field)
#define SW_LIST_ADD(head, node, field)              list_add_tail(&node->field, head)
#define SW_LIST_GET_HEAD_ENTRY(head, type, field)   list_first_entry(head, struct type, field)
#define SW_LIST_UNLINK(node, field)                 list_del(&node->field)
#define SW_LIST_FOR_EACH_ENTRY(node, head, field)   list_for_each_entry(node, head, field)
#define SW_LIST_EMPTY(head)                         list_empty(head)
#define SW_LIST_HEAD_INITIALIZER(head)              LIST_HEAD_INIT(head)

#endif // __SW_LIST_H__
