        AREA helloW, CODE, READONLY  ; ����һ����ΪhelloW��ֻ�������
        ENTRY                       ; ������ڵ�
        EXPORT __main               ; Keil��Ҫ������������

SWI_WriteC EQU 0x0                 ; �����ַ��������жϺ�
SWI_Exit   EQU 0x11                ; ��������˳�����жϺ�

__main
START
LOOP
        ADR r1, TEXT               ; ��TEXT��ǩ�ĵ�ַ���ص�r1
        LDRB r0, [r1], #1          ; ��r1ָ��ĵ�ַ����һ���ֽڵ�r0��Ȼ��r1��1
        CMP r0, #0                 ; �Ƚ�r0��0(null�ַ�)
        SWINE SWI_WriteC           ; ���r0��Ϊ0������SWI_WriteC����ַ�
        BNE LOOP                   ; ���r0��Ϊ0������ѭ��
        SWI SWI_Exit               ; ����SWI_Exit��������

TEXT
        DCB "Hello World!", 0x0A, 0x0D, 0  ; ʹ��DCB�����ַ���
        ALIGN                          ; ȷ����ַ����

        END                           ; �������