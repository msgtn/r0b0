import { useEffect, useRef } from 'react';
import * as React from 'react';

export type FileInputProps = {
  fileList: File[];
  onChange(fileList: FileList): void;
};
const FileInput = ({ fileList = [], onChange }: FileInputProps) => {
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (inputRef.current) {
      const dataTransfer = new DataTransfer();
      fileList.forEach((file) => dataTransfer.items.add(file));
      inputRef.current.files = dataTransfer.files;
    }
  }, [fileList]);

  return (
    <input
      type="file"
      ref={inputRef}
      data-testid="uploader"
      onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
        onChange(e.target.files);
      }}
    />
  );
};

export default FileInput;