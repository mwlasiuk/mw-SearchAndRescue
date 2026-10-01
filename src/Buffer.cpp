#include <cave-traversal-tool/OpenGL/Buffer.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

struct Buffer::BufferIMPL
{
    uint32_t id    = {};
    uint32_t usage = {};
    size_t   size  = {};
};

Buffer::Buffer(const uint32_t usage, const size_t size, const void* data)
{
    _impl = new BufferIMPL;

    _impl->id    = UINT32_MAX;
    _impl->usage = usage;
    _impl->size  = size;

    int32_t previous_binding = 0;
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &previous_binding);
    glGenBuffers(1, &_impl->id);
    glBindBuffer(GL_ARRAY_BUFFER, _impl->id);
    glBufferData(GL_ARRAY_BUFFER, _impl->size, data, _impl->usage);
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<uint32_t>(previous_binding));
}

Buffer::~Buffer()
{
    glDeleteBuffers(1, &_impl->id);

    delete _impl;
}

uint32_t Buffer::GetID() const
{
    return _impl->id;
}

uint32_t Buffer::GetUsage() const
{
    return _impl->usage;
}

size_t Buffer::GetSize() const
{
    return _impl->size;
}

void Buffer::Upload(const void* data, const size_t size, const size_t offset) const
{
    if (_impl->usage != GL_DYNAMIC_DRAW)
    {
        spdlog::critical("Buffer ID = {} was not created for dynamic updates - upload rejected!", _impl->id);
        return;
    }

    if (offset > _impl->size || size > (_impl->size - offset))
    {
        spdlog::critical("Buffer ID = {} size = {} bytes - attempting to upload {} bytes from {:p} at offset {} - upload rejected!", _impl->id, _impl->size, size, static_cast<const void*>(data), offset);
        return;
    }

    int32_t previous_binding = 0;
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &previous_binding);
    glBindBuffer(GL_ARRAY_BUFFER, _impl->id);
    glBufferSubData(GL_ARRAY_BUFFER, offset, size, data);
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<uint32_t>(previous_binding));
}

void Buffer::SetAsShaderResource(const uint32_t resource_type, const uint32_t binding, const size_t size, const size_t offset) const
{
    if (offset > _impl->size || size > (_impl->size - offset))
    {
        spdlog::critical("Buffer ID = {} size = {} - invalid bind range size = {} offset = {} - bind rejected!", _impl->id, _impl->size, size, offset);
        return;
    }

    glBindBufferRange(resource_type, binding, _impl->id, offset, size);
}