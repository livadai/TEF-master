        AREA helloW, CODE, READONLY  ; 声明一个名为helloW的只读代码段
        ENTRY                       ; 程序入口点
        EXPORT __main               ; Keil需要的主函数声明

SWI_WriteC EQU 0x0                 ; 定义字符输出软件中断号
SWI_Exit   EQU 0x11                ; 定义程序退出软件中断号

__main
START
LOOP
        ADR r1, TEXT               ; 将TEXT标签的地址加载到r1
        LDRB r0, [r1], #1          ; 从r1指向的地址加载一个字节到r0，然后r1加1
        CMP r0, #0                 ; 比较r0和0(null字符)
        SWINE SWI_WriteC           ; 如果r0不为0，调用SWI_WriteC输出字符
        BNE LOOP                   ; 如果r0不为0，继续循环
        SWI SWI_Exit               ; 调用SWI_Exit结束程序

TEXT
        DCB "Hello World!", 0x0A, 0x0D, 0  ; 使用DCB定义字符串
        ALIGN                          ; 确保地址对齐

        END                           ; 程序结束