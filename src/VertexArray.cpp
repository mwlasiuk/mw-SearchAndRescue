#include <cave-traversal-tool/OpenGL/VertexArray.h>

// clang-format off
#include <spdlog/spdlog.h>
#include <glad/glad.h>
// clang-format on

struct VertexArray::VertexArrayIMPL
{
    uint32_t id = {};

    Buffer* vbo           = {};
    bool    vbo_ownership = {};

    Buffer* ibo           = {};
    bool    ibo_ownership = {};
};

VertexArray::VertexArray(Buffer* vertex_buffer, const bool vertex_buffer_ownership, Buffer* index_buffer, const bool index_buffer_ownership, const std::vector<VertexBufferAttributeLayout>& layout)
{
    _impl = new VertexArrayIMPL;

    _impl->id            = UINT32_MAX;
    _impl->vbo           = vertex_buffer;
    _impl->vbo_ownership = vertex_buffer_ownership;
    _impl->ibo           = index_buffer;
    _impl->ibo_ownership = index_buffer_ownership;

    int32_t previous_vao           = 0;
    int32_t previous_array_binding = 0;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previous_vao);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &previous_array_binding);
    glGenVertexArrays(1, &_impl->id);
    glBindVertexArray(_impl->id);

    if (_impl->ibo)
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _impl->ibo->GetID());
    }

    if (_impl->vbo)
    {
        glBindBuffer(GL_ARRAY_BUFFER, _impl->vbo->GetID());
        for (const VertexBufferAttributeLayout& attribute_layout : layout)
        {
            const auto& [location, components, type, normalize, stride, offset] = attribute_layout;

            glVertexAttribPointer(location, components, type, static_cast<uint8_t>(normalize), stride, reinterpret_cast<const void*>(static_cast<uintptr_t>(offset)));
            glEnableVertexAttribArray(location);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, static_cast<uint32_t>(previous_array_binding));
    glBindVertexArray(static_cast<uint32_t>(previous_vao));
}

VertexArray::~VertexArray()
{
    if (_impl->ibo_ownership && _impl->ibo)
    {
        delete _impl->ibo;
    }

    if (_impl->vbo_ownership && _impl->vbo)
    {
        delete _impl->vbo;
    }

    glDeleteVertexArrays(1, &_impl->id);

    delete _impl;
}

uint32_t VertexArray::GetID() const
{
    return _impl->id;
}

void VertexArray::Bind()
{
    glBindVertexArray(_impl->id);
}

void VertexArray::Unbind()
{
    glBindVertexArray(0);
}

void VertexArray::DrawArray(const uint32_t mode, const uint32_t vertex_count)
{
    glDrawArrays(mode, 0, vertex_count);
}

void VertexArray::DrawArray(const uint32_t mode, const uint32_t first, const uint32_t vertex_count)
{
    glDrawArrays(mode, first, vertex_count);
}

void VertexArray::DrawElements(const uint32_t mode, const uint32_t index_count)
{
    glDrawElements(mode, index_count, GL_UNSIGNED_INT, nullptr);
}
