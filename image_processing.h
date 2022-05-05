#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40

void update_line_position(uint8_t *buffer);
void image_processing_start(void);
uint16_t get_line_position(void);

#endif /* PROCESS_IMAGE_H */
